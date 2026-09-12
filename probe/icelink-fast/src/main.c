/* icelink-fast -- ECP5 JTAG probe.
 *
 * This build exists to answer one question: does the JTAG engine read a valid
 * IDCODE from the i9's ECP5? The LED is physically covered by the SODIMM once
 * it is seated, and the SWD grabbers are off, so the answer has to come out
 * over USB. Hence CDC.
 */
#include "apm32f103.h"
#include "tusb.h"
#include "uart.h"
#include "ecp5.h"

#define LED_A    0u    /* PB0 -- LED (covered by the module, kept for parity) */
#define LED_B    6u
#define USB_ENUM 15u   /* PB15 gates the USB D+ pull-up; nothing enumerates
                        * without it (from the board schematic). */

void clock_init(void);
void jtag_init(void);
uint32_t jtag_read_idcode(void);
uint32_t jtag_read_id_via_ir(void);
uint32_t jtag_read_usercode(void);
uint32_t jtag_read_status(void);
uint32_t jtag_bench_cycles_per_bit_x256(void);
int jtag_verify(unsigned rounds);
void jtag_bus_release(void);
uint32_t jtag_capture(uint8_t *buf, uint32_t n, uint32_t timeout);
void jtag_monitor(uint32_t iters, uint32_t *etck, uint32_t *etms,
                  uint32_t *etdi, uint32_t *etdo, uint32_t *hi, uint32_t *lo);
void jtag_bus_acquire(void);
extern uint32_t g_jtag_delay;
extern uint32_t g_bus_released;
void enter_bootloader(void);
void request_bootloader(void);

extern uint32_t g_idcode, g_idcode_tries, g_expect_idcode;
extern uint32_t g_sysclk_hz;

/* If USB never comes up, put ourselves back in the bootloader rather than
 * stranding the board: with no grabbers and no visible LED, an image that
 * cannot be talked to would otherwise be unrecoverable. */
#define DEADMAN_MS 20000u

/* CDC 0 = this console, CDC 1 = transparent bridge to the target's USART2.
 * Default baud matches the LiteX SoC's uart_baudrate, so litex_term works even
 * if the host never sends a SET_LINE_CODING. */
#define CDC_CONSOLE       0
#define CDC_UART          1
#define UART_DEFAULT_BAUD 1000000u

/* The host owns the baud rate: whatever litex_term (or any terminal) asks for
 * is what we put on the wire. */
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *coding)
{
    if (itf == CDC_UART) uart_set_baud(coding->bit_rate);
}

/* Move bytes both ways between CDC 1 and USART2. Never blocks: each direction
 * only moves what both the ring and the USB FIFO can take right now, and
 * whatever is left waits for the next pass. Anything that spun here would
 * stall tud_task() and stutter the console. */
static void uart_bridge_task(void)
{
    uint8_t buf[64];

    /* USB -> UART. Clamp to the TX ring's free space: anything we pull out of
     * the CDC FIFO must fit, because there is nowhere else to put it. Spinning
     * until it fits instead would block for however long the wire needs --
     * 64 bytes is ~640us at 1 Mbaud but ~67ms at 9600, and tud_task() does not
     * run for the duration. Whatever does not fit stays in the CDC FIFO and
     * comes back on the next pass, which also back-pressures the host. */
    uint32_t space = uart_tx_space();
    if (space > sizeof buf) space = sizeof buf;
    uint32_t n = tud_cdc_n_available(CDC_UART);
    if (n > space) n = space;
    if (n) {
        n = tud_cdc_n_read(CDC_UART, buf, n);
        uart_write(buf, n);          /* fits by construction */
    }

    /* UART -> USB. */
    uint32_t room = tud_cdc_n_write_available(CDC_UART);
    if (room) {
        if (room > sizeof buf) room = sizeof buf;
        uint32_t got = uart_read(buf, room);
        if (got) {
            tud_cdc_n_write(CDC_UART, buf, got);
            tud_cdc_n_write_flush(CDC_UART);
        }
    }
}

static uint32_t millis_elapsed(void)
{
    /* DWT CYCCNT wraps every ~60 s at 72 MHz, so accumulate deltas. */
    static uint32_t last, acc_ms, acc_cyc;
    uint32_t now = DWT_CYCCNT;
    uint32_t delta = now - last;      /* unsigned wrap is well-defined */
    last = now;

    uint32_t per_ms = g_sysclk_hz / 1000u;
    if (per_ms == 0u) per_ms = 72000u;
    acc_cyc += delta;
    while (acc_cyc >= per_ms) {
        acc_cyc -= per_ms;
        acc_ms++;
    }
    return acc_ms;
}

static void usb_hw_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    /* USB_LP_CAN1_RX0 is IRQ 20 on the F103. */
    NVIC_ISER(20u / 32u) = 1u << (20u % 32u);

    tusb_init();

    /* Assert the pull-up only once the device stack is ready, so the host's
     * first descriptor request cannot arrive before we can answer it. */
    gpio_cfg(GPIOB, USB_ENUM, GPIO_MODE_OUT_PP_50);
    GPIOB->BSRR = (1u << USB_ENUM);
}

static char obuf[256];
static int olen;

static void emit(const char *s)
{
    while (*s && olen < (int)sizeof(obuf) - 1) obuf[olen++] = *s++;
}

static void emit_hex32(uint32_t v)
{
    static const char hex[] = "0123456789abcdef";
    for (int i = 28; i >= 0; i -= 4)
        if (olen < (int)sizeof(obuf) - 1) obuf[olen++] = hex[(v >> i) & 0xF];
}

static void emit_dec(uint32_t v)
{
    char t[12];
    int n = 0;
    if (v == 0) t[n++] = '0';
    while (v) { t[n++] = (char)('0' + v % 10u); v /= 10u; }
    while (n && olen < (int)sizeof(obuf) - 1) obuf[olen++] = t[--n];
}

/* Write everything, waiting for FIFO space. tud_cdc_write() silently drops
 * whatever does not fit (256-byte TX FIFO), which truncated the first capture
 * dump -- so push in chunks and keep the stack running while we wait. */
static void flush_out(void)
{
    int sent = 0;
    while (sent < olen) {
        uint32_t space = tud_cdc_write_available();
        if (space == 0) { tud_task(); continue; }
        uint32_t chunk = (uint32_t)(olen - sent);
        if (chunk > space) chunk = space;
        sent += (int)tud_cdc_write(obuf + sent, chunk);
        tud_cdc_write_flush();
        tud_task();
    }
    olen = 0;
}

static const char *part_name(uint32_t id)
{
    switch (id) {
    case 0x41112043u: return "LFE5U-45F";
    case 0x41111043u: return "LFE5U-25F";
    case 0x41113043u: return "LFE5U-85F";
    case 0x21111043u: return "LFE5U-12F";
    case 0u:          return "TDO stuck LOW";
    case 0xFFFFFFFFu: return "TDO stuck HIGH";
    default:          return "unknown";
    }
}

static void print_speed_line(uint32_t delay, uint32_t cpb256, int ok)
{
    olen = 0;
    emit("  delay="); emit_dec(delay);
    emit("  cycles/bit="); emit_dec(cpb256 / 256u);
    emit("."); emit_dec(((cpb256 % 256u) * 100u) / 256u);
    uint32_t khz = (uint32_t)(((uint64_t)g_sysclk_hz * 256u) / cpb256 / 1000u);
    emit("  TCK="); emit_dec(khz); emit(" kHz");
    emit("  "); emit_dec(khz / 8u); emit(" kB/s  ");
    emit(ok ? "IDCODE OK" : "IDCODE FAIL");
    emit("\r\n");
    flush_out();
}

static void speed_sweep(void)
{
    olen = 0;
    emit("\r\n--- speed sweep (fastest that still reads IDCODE) ---\r\n");
    flush_out();

    uint32_t saved = g_jtag_delay;
    uint32_t best = 0xFFFFFFFFu;

    /* Establish what a correct read looks like on THIS part, at the current
     * (known-working) speed, before trusting any faster one. */
    g_expect_idcode = jtag_read_idcode();
    if (g_expect_idcode == 0u || g_expect_idcode == 0xFFFFFFFFu) {
        olen = 0;
        emit("no valid IDCODE at the current speed (0x"); emit_hex32(g_expect_idcode);
        emit(") -- is the FPGA seated?\r\n");
        flush_out();
        return;
    }
    olen = 0;
    emit("verifying against 0x"); emit_hex32(g_expect_idcode);
    emit("  "); emit(part_name(g_expect_idcode)); emit("\r\n");
    flush_out();

    for (int32_t d = 8; d >= 0; d--) {
        g_jtag_delay = (uint32_t)d;
        uint32_t cpb = jtag_bench_cycles_per_bit_x256();
        int ok = jtag_verify(32);
        print_speed_line((uint32_t)d, cpb, ok);
        if (ok) best = (uint32_t)d;
        tud_task();
    }

    g_jtag_delay = (best == 0xFFFFFFFFu) ? saved : best;
    olen = 0;
    emit("selected delay = "); emit_dec(g_jtag_delay); emit("\r\n");
    flush_out();
}

/* Read exactly n bytes from the console CDC, pumping the USB stack while we
 * wait. Returns 0 if the host goes quiet for timeout_ms. */
static int con_read_exact(uint8_t *dst, uint32_t n, uint32_t timeout_ms)
{
    uint32_t got = 0, t0 = millis_elapsed();
    while (got < n) {
        tud_task();
        uint32_t avail = tud_cdc_available();
        if (avail) {
            uint32_t want = n - got;
            if (want > avail) want = avail;
            got += tud_cdc_read(dst + got, want);
            t0 = millis_elapsed();
        } else if (millis_elapsed() - t0 > timeout_ms) {
            return 0;
        }
    }
    return 1;
}

/* 'p': stream a bitstream into the ECP5's SRAM.
 *
 * Wire format is deliberately trivial -- u32le length, then that many bytes --
 * because the bitstream cannot be buffered here (1.4 MB of payload against
 * 20 KB of RAM). Bytes go from the USB FIFO to TDI as they arrive. */
static void bitstream_load(void)
{
    uint8_t hdr[4];
    olen = 0; emit("\r\nsend u32le length, then the bitstream\r\n"); flush_out();

    if (!con_read_exact(hdr, 4, 10000u)) {
        olen = 0; emit("timeout waiting for length\r\n"); flush_out(); return;
    }
    uint32_t len = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) |
                   ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
    if (len == 0u || len > 8u * 1024u * 1024u) {
        olen = 0; emit("implausible length 0x"); emit_hex32(len);
        emit("\r\n"); flush_out(); return;
    }

    uint32_t idcode = 0, st_erase = 0;
    ecp5_err_t e = ecp5_config_begin(&idcode, &st_erase);
    olen = 0;
    emit("IDCODE 0x"); emit_hex32(idcode); emit("  "); emit(part_name(idcode));
    emit("\r\nstatus after erase = 0x"); emit_hex32(st_erase);
    emit((st_erase & ECP5_STATUS_DONE) ? "  (DONE still set?!)" : "  DONE cleared");
    emit("\r\n"); flush_out();
    if (e != ECP5_OK) {
        olen = 0; emit("config_begin failed -- is the FPGA seated?\r\n");
        flush_out(); return;
    }

    olen = 0; emit("shifting "); emit_dec(len); emit(" bytes...\r\n"); flush_out();

    uint8_t buf[256];
    uint32_t remaining = len, t_start = millis_elapsed(), t0 = t_start;
    while (remaining) {
        tud_task();
        uint32_t avail = tud_cdc_available();
        if (!avail) {
            if (millis_elapsed() - t0 > 5000u) {
                olen = 0; emit("host stalled with "); emit_dec(remaining);
                emit(" bytes to go -- aborting\r\n"); flush_out();
                (void)ecp5_config_end(0);
                return;
            }
            continue;
        }
        uint32_t want = (remaining < sizeof buf) ? remaining : (uint32_t)sizeof buf;
        if (want > avail) want = avail;
        uint32_t got = tud_cdc_read(buf, want);
        ecp5_config_data(buf, got);
        remaining -= got;
        t0 = millis_elapsed();
    }

    uint32_t st = 0;
    e = ecp5_config_end(&st);
    uint32_t ms = millis_elapsed() - t_start;

    olen = 0;
    emit("final status = 0x"); emit_hex32(st);
    emit(e == ECP5_OK ? "  DONE -- configured\r\n" : "  DONE NOT SET -- failed\r\n");
    emit("took "); emit_dec(ms); emit(" ms");
    if (ms) { emit(", "); emit_dec(len / ms); emit(" kB/s"); }
    emit("\r\n");
    flush_out();
}

static void print_report(void)
{
    if (g_bus_released) {
        olen = 0;
        emit("\r\nJTAG bus RELEASED (hi-Z) -- external programmer has it.\r\n");
        emit("  TCK=PB13/pin26  TMS=PB14/pin27  TDI=PB8/pin45  TDO=PB9/pin46\r\n");
        emit("  press 'a' to take the bus back\r\n");
        flush_out();
        return;
    }

    uint32_t id_dr  = jtag_read_idcode();      /* post-reset default DR */
    uint32_t id_ir  = jtag_read_id_via_ir();   /* via READ_ID -- proves IR works */
    uint32_t user   = jtag_read_usercode();
    uint32_t status = jtag_read_status();

    g_idcode = id_dr;
    g_idcode_tries++;

    olen = 0;
    emit("\r\n--- ECP5 ---\r\n");
    emit("IDCODE  (DR default) = 0x"); emit_hex32(id_dr);
    emit("  "); emit(part_name(id_dr)); emit("\r\n");
    emit("IDCODE  (via READ_ID)= 0x"); emit_hex32(id_ir);
    emit(id_ir == id_dr ? "  IR path OK\r\n" : "  MISMATCH -- IR path suspect\r\n");
    emit("USERCODE             = 0x"); emit_hex32(user); emit("\r\n");
    emit("STATUS               = 0x"); emit_hex32(status); emit("\r\n");
    flush_out();

    /* Speed. cycles-per-bit is measured, TCK is derived from the real SYSCLK. */
    uint32_t cpb256 = jtag_bench_cycles_per_bit_x256();
    uint32_t tck_khz = (uint32_t)(((uint64_t)g_sysclk_hz * 256u) / cpb256 / 1000u);

    olen = 0;
    emit("TCK cycles/bit       = "); emit_dec(cpb256 / 256u);
    emit("."); emit_dec(((cpb256 % 256u) * 100u) / 256u); emit("\r\n");
    emit("TCK rate             = "); emit_dec(tck_khz); emit(" kHz\r\n");
    emit("shift throughput     = "); emit_dec(tck_khz / 8u); emit(" kB/s\r\n");
    emit("target UART          = "); emit_dec(uart_get_baud());
    emit(" baud on if02");
    if (uart_rx_dropped()) { emit("  DROPPED "); emit_dec(uart_rx_dropped()); }
    emit("\r\n");
    emit("cmds: b=bootloader r=re-read s=speed sweep z=hi-Z a=acquire m=monitor\r\n");
    emit("      c=capture p=program bitstream (u32le len + data)\r\n");
    flush_out();
}

int main(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    gpio_cfg(GPIOB, LED_A, GPIO_MODE_OUT_PP_50);
    gpio_cfg(GPIOB, LED_B, GPIO_MODE_OUT_PP_50);

    clock_init();
    uart_init(UART_DEFAULT_BAUD);

    jtag_init();
    g_idcode = jtag_read_idcode();
    g_idcode_tries++;

    usb_hw_init();

    uint32_t last_print = 0;
    int reported = 0;

    for (;;) {
        tud_task();
        uart_bridge_task();

        uint32_t ms = millis_elapsed();

        if (!tud_mounted() && ms > DEADMAN_MS) {
            /* Nobody is listening and nobody can see the LED. Go back to the
             * bootloader so the next image can be flashed over USB. */
            enter_bootloader();
        }

        /* Host commands. 'b' finally removes the last need for SWD: it puts
         * us back in the bootloader on demand instead of only when the
         * dead-man timer fires. */
        if (tud_cdc_available()) {
            char c;
            if (tud_cdc_read(&c, 1) == 1) {
                if (c == 'b' || c == 'B') {
                    olen = 0;
                    emit("\r\nentering bootloader -- MAINTENANCE will appear\r\n");
                    flush_out();
                    for (volatile uint32_t d = 0; d < 2000000u; d++) { } /* let it drain */
                    request_bootloader();
                } else if (c == 'r' || c == 'R') {
                    reported = 0;
                } else if (c == 's' || c == 'S') {
                    if (!g_bus_released) speed_sweep();
                    reported = 0;
                } else if (c == 'z' || c == 'Z') {
                    jtag_bus_release();
                    olen = 0;
                    emit("\r\nJTAG pins are now hi-Z. Clip the CH347 on:\r\n");
                    emit("  TCK=PB13/pin26  TMS=PB14/pin27  TDI=PB8/pin45  TDO=PB9/pin46\r\n");
                    emit("  share GND. openFPGALoader -c ch347 ...\r\n");
                    flush_out();
                    reported = 1;
                    last_print = ms;
                } else if (c == 'm' || c == 'M') {
                    if (!g_bus_released) jtag_bus_release();
                    olen = 0;
                    emit("\r\nmonitoring nets for ~2 s -- drive them now\r\n");
                    flush_out();
                    uint32_t etck, etms, etdi, etdo, hi, lo;
                    jtag_monitor(6000000u, &etck, &etms, &etdi, &etdo, &hi, &lo);
                    olen = 0;
                    emit("edges TCK(PB13)="); emit_dec(etck);
                    emit("  TMS(PB14)="); emit_dec(etms);
                    emit("\r\nedges TDI(PB8) ="); emit_dec(etdi);
                    emit("  TDO(PB9) ="); emit_dec(etdo);
                    emit("\r\never-high mask=0x"); emit_hex32(hi);
                    emit("  always-high mask=0x"); emit_hex32(lo);
                    emit("\r\n(PB8=0x100 PB9=0x200 PB13=0x2000 PB14=0x4000)\r\n");
                    flush_out();
                    reported = 1; last_print = ms;
                } else if (c == 'c' || c == 'C') {
                    if (!g_bus_released) jtag_bus_release();
                    static uint8_t cap[1024];
                    olen = 0; emit("\r\ncapturing 1024 TCK edges...\r\n"); flush_out();
                    uint32_t got = jtag_capture(cap, sizeof(cap), 80000000u);
                    olen = 0; emit("samples="); emit_dec(got); emit("\r\n"); flush_out();
                    /* Where does TMS actually move? That is the whole question. */
                    olen = 0; emit("TMS rising edges at TCK #: ");
                    for (uint32_t k = 1; k < got; k++)
                        if ((cap[k] & 1u) && !(cap[k-1] & 1u)) {
                            emit_dec(k); emit(" ");
                            if (olen > 180) flush_out();
                        }
                    emit("\r\n"); flush_out();

                    for (int row = 0; row < 3; row++) {
                        emit(row == 0 ? "TMS " : (row == 1 ? "TDI " : "TDO "));
                        for (uint32_t k = 0; k < got; k++) {
                            if (olen >= 64) flush_out();
                            obuf[olen++] = ((cap[k] >> row) & 1u) ? '1' : '0';
                        }
                        emit("\r\n");
                        flush_out();
                    }
                    reported = 1; last_print = ms;
                } else if (c == 'p' || c == 'P') {
                    if (g_bus_released) jtag_bus_acquire();
                    bitstream_load();
                    reported = 0;
                } else if (c == 'a' || c == 'A') {
                    jtag_bus_acquire();
                    olen = 0;
                    emit("\r\nJTAG bus re-acquired.\r\n");
                    flush_out();
                    reported = 0;
                }
            }
        }

        if (tud_cdc_connected() && (!reported || ms - last_print > 3000u) &&
            !g_bus_released) {
            print_report();
            last_print = ms;
            reported = 1;
        }

        GPIOB->BSRR = (ms & 0x100u) ? (1u << LED_A) : (1u << (LED_A + 16));
    }
}
