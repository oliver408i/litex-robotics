/* ESP32-C3 SPIBone flash loader -- C3 side ("brain").
 *
 * Drives the FPGA as a Wishbone master over SPIBone (HW SPI @8MHz, the reliable
 * production clock from the sweep) and speaks the mailbox protocol in
 * software/c3_flash/main.c. The C3 writes a command + page data into the uncached
 * mailbox RAM (@0x90000000) and rings a doorbell; the FPGA firmware programs
 * flash and returns status/result. See icepi_zero_c3flash.py.
 *
 * STAGE 1: PING -- read the flash JEDEC id back through the mailbox, proving the
 * whole chain (C3 -> SPIBone -> mailbox -> firmware -> LiteSPI master -> flash).
 *
 * Wiring: SCLK=GPIO8/P3  MOSI=GPIO1/L1  MISO=GPIO3/M2  CS#=GPIO2/J3.
 * SPIBone 4-wire, mode 0, big-endian; SPIBone shifts the byte address internally,
 * so we send full BYTE addresses. Markers: 0x01 read reply / 0x00 write ack.
 */
#include <Arduino.h>
#include <SPI.h>

#define PIN_SCLK   8
#define PIN_MOSI   1
#define PIN_MISO   3
#define PIN_CS     2

/* C3 -> FPGA reset line: pulse low to reset the FPGA's CPU/clock domains via
 * its ext_reset input (icepi_zero_base.py, ball G3, active-low, pulled up on
 * the FPGA side -- no gateware changes needed, see [[c3-loader-production-scope]]).
 * GPIO7 chosen deliberately: NOT a strapping pin (2/8/9), NOT UART0 (20/21 --
 * both glitch this line during the C3's own boot/reset and would reset the
 * FPGA every time the C3 resets, see [[c3-reset-line-gpio21-gotcha]]), NOT the
 * native-USB D+/D- pins (18/19), and NOT GPIO10 (reserved for future direct
 * FPGA communication). Driven open-drain style (only ever OUTPUT+LOW or
 * INPUT) so it never fights the FPGA-side pull-up. */
#define PIN_RESET  7
#define RESET_PULSE_MS 10

/* C3 -> FPGA boot-mode strap: the old SPISlave READY wire (C3 GPIO10 <-> FPGA
 * IO4/R1, unused by SPIBone -- GPIO10 was reserved for "future direct FPGA
 * communication", this is it). Tells the resident loader whether to stay for
 * flashing or chain-boot the app (gateware loader_stay GPIOIn, pulled up FPGA
 * side; loader stay_requested()). Open-drain like PIN_RESET: OUTPUT+LOW =
 * request STAY; INPUT/Hi-Z = release -> FPGA reads high -> chain-boot the app. */
#define PIN_STAY   10

#define SPI_HZ     8000000UL     /* production clock: reliable, ~2x the flash ceiling */
#define POLL_MAX   256

/* Windowed ACK for the host image-streaming protocol: ack every WINDOW_PAGES
 * (4 KB) instead of every single 256 B page -- profiling (2026-07-03) showed
 * the per-page USB round-trip was ~26% of total transfer time. Must match
 * flash_c3.py's WINDOW_PAGES. RX_BUFFER_SIZE must comfortably exceed one
 * window's worth of bytes -- see setup(). */
#define WINDOW_PAGES     16
#define RX_BUFFER_SIZE   8192

/* Mailbox (must match gateware add_c3_mailbox + software/c3_flash). */
#define MBX_BASE   0x90000000UL
#define MBX_CMD    0x00
#define MBX_ARG0   0x04
#define MBX_ARG1   0x08
#define MBX_STATUS 0x0C
#define MBX_RESULT 0x10
#define MBX_DATA   0x40

#define CMD_PING     0x01
#define CMD_ERASE    0x02
#define CMD_PROGRAM  0x03
#define CMD_CRC      0x04
#define CMD_REBOOT   0x05
#define CMD_BOOT_APP 0x06
#define CMD_STAY     0x07   /* set stay flag + ctrl_reset -> stay resident (must match software/c3_flash) */
#define ST_OK        0x00

/* SAFE scratch sector for the self-test: 15 MB into the 16 MB W25Q128, far above
 * the bitstream (0), BIOS (0x100000) and app slot (0x280000). This region IS
 * erased/programmed -- pick something known-unused. */
#define TEST_OFF     0x00F00000UL

/* Match LiteX libbase crc32 exactly (init 0xFFFFFFFF, reflected 0xEDB88320,
 * final XOR) so our expected CRC equals the firmware's read-back CRC. */
static uint32_t crc32_c3(const uint8_t *msg, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= msg[i];
        for (int j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
    }
    return ~crc;
}

SPIClass spi(FSPI);

static inline void cs_lo() { digitalWrite(PIN_CS, LOW); }
static inline void cs_hi() { digitalWrite(PIN_CS, HIGH); }

static int wait_marker() {
    for (int i = 0; i < POLL_MAX; i++) {
        uint8_t b = spi.transfer(0x00);
        if (b != 0xFF) return b;
    }
    return -1;
}

/* RAII SPI transaction guard. wb_read/wb_write used to open+close their own
 * SPI transaction (clock/mode reconfiguration) on every single 32-bit word --
 * fine in isolation, but a mailbox command fires 60+ of them back-to-back
 * (arg0/arg1/64 data words/doorbell, then the doorbell-poll reads), and
 * profiling (2026-07-03, real 640 KB transfer) showed that per-word overhead
 * eating ~38% of total time. One transaction spans a whole mbx_cmd() instead;
 * CS# still toggles per word (SPIBone's FSM resets on cs_n -- one word per
 * CS-assert cycle is a hard protocol requirement, not just an optimization
 * target -- see litex's spi_bone.py), only the SPI peripheral reconfig is hoisted. */
struct SPISession {
    SPISession()  { spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0)); }
    ~SPISession() { spi.endTransaction(); }
};

/* --- SPIBone Wishbone primitives (byte address; SPIBone shifts internally).
 * Caller must hold an open SPISession. --- */
static bool wb_read(uint32_t addr, uint32_t *val) {
    bool ok = false;
    cs_lo();
    spi.transfer(0x01);
    spi.transfer((addr >> 24) & 0xFF); spi.transfer((addr >> 16) & 0xFF);
    spi.transfer((addr >>  8) & 0xFF); spi.transfer((addr >>  0) & 0xFF);
    if (wait_marker() == 0x01) {
        uint32_t d = 0;
        for (int i = 0; i < 4; i++) d = (d << 8) | spi.transfer(0x00);
        *val = d; ok = true;
    }
    cs_hi();
    return ok;
}

static bool wb_write(uint32_t addr, uint32_t val) {
    cs_lo();
    spi.transfer(0x00);
    spi.transfer((addr >> 24) & 0xFF); spi.transfer((addr >> 16) & 0xFF);
    spi.transfer((addr >>  8) & 0xFF); spi.transfer((addr >>  0) & 0xFF);
    spi.transfer((val >> 24) & 0xFF);  spi.transfer((val >> 16) & 0xFF);
    spi.transfer((val >>  8) & 0xFF);  spi.transfer((val >>  0) & 0xFF);
    bool ok = (wait_marker() == 0x00);
    cs_hi();
    return ok;
}

/* Run a mailbox command. Writes arg0/arg1 (+ optional page data), rings the
 * doorbell (cmd), waits for the firmware to clear it, returns status (or <0 on
 * link/timeout error) and *result. `data`/`n` used by PROGRAM (n<=256). */
static int mbx_cmd(uint8_t op, uint32_t arg0, uint32_t arg1,
                   const uint8_t *data, uint32_t n, uint32_t *result,
                   uint32_t timeout_ms = 5000) {
    SPISession _session;   /* one SPI transaction spans the whole command */
    if (!wb_write(MBX_BASE + MBX_ARG0, arg0)) return -10;
    if (!wb_write(MBX_BASE + MBX_ARG1, arg1)) return -10;
    /* Pack payload little-endian: word at DATA+4k -> RAM bytes [4k..4k+3], so
     * RAM byte i is flash byte i (firmware reads it as a byte array). */
    for (uint32_t i = 0; i < n; i += 4) {
        uint32_t w = 0;
        for (uint32_t b = 0; b < 4 && i + b < n; b++)
            w |= (uint32_t)data[i + b] << (8 * b);
        if (!wb_write(MBX_BASE + MBX_DATA + i, w)) return -10;
    }
    if (!wb_write(MBX_BASE + MBX_CMD, op)) return -10;    /* doorbell */

    uint32_t tm0 = millis(), c = 0xffffffff;
    do {
        if (!wb_read(MBX_BASE + MBX_CMD, &c)) return -11;
        if (millis() - tm0 > timeout_ms) return -12;      /* firmware never cleared cmd */
    } while (c != 0);

    uint32_t status = 0xff;
    if (!wb_read(MBX_BASE + MBX_STATUS, &status)) return -11;
    if (result && !wb_read(MBX_BASE + MBX_RESULT, result)) return -11;
    return (int)status;
}

/* Fire-and-forget doorbell: rings the mailbox with no arg writes and no
 * completion poll -- for opcodes where the firmware resets itself right after
 * seeing the doorbell (CMD_BOOT_APP / CMD_STAY / CMD_REBOOT) and so never clears
 * MBX_CMD; mbx_cmd()'s poll loop would just time out. This is the PRIMARY boot-
 * control reset for 'l'/'b'/'R': the loader reboots via its ctrl_reset CSR (a
 * SoCController soft reset) plus the durable boot_ctl flag, which does NOT depend
 * on the external C3->FPGA reset line (PIN_RESET/G3). That line is kept only as a
 * fallback pulse below -- see docs/c3_loader.md and [[c3-reset-line-gpio21-gotcha]].
 * Returns true if the doorbell write reached the FPGA (loader present). */
static bool mbx_fire(uint8_t op) {
    SPISession _session;
    return wb_write(MBX_BASE + MBX_CMD, op);
}

void setup() {
    /* Native USB-Serial-JTAG RX has NO backpressure: HWCDC.cpp pulls bytes
     * from the hardware FIFO in an ISR and silently DROPS whatever doesn't
     * fit once the rx_queue is full (no NAK, no stall). Must be set before
     * begin(); must stay >= a couple of WINDOW_PAGES worth of bytes so a
     * whole window arriving while a page's flash-program blocks the drain
     * loop can't overflow it. */
    Serial.setRxBufferSize(RX_BUFFER_SIZE);
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) ;

    pinMode(PIN_CS, OUTPUT);
    cs_hi();
    spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);

    pinMode(PIN_RESET, INPUT);   /* idle: high-Z, FPGA-side pull-up holds it high */
    pinMode(PIN_STAY,  INPUT);   /* idle: released -> FPGA reads high -> boot app */

    Serial.println("\n[c3] SPIBone flash loader");
    Serial.printf("[c3] SPI @%lu Hz, mailbox @0x%08lX. cmds: 't'=self-test  'p'=ping"
                  "  'R'=pulse FPGA reset  'l'=enter loader (stay)  'b'=boot app"
                  "  'W..'=host image stream (flash_c3.py)\n",
                  (unsigned long)SPI_HZ, (unsigned long)MBX_BASE);
}

/* Pulse the FPGA's ext_reset line low, open-drain style: only ever drive LOW
 * or release to high-Z INPUT, never drive HIGH (avoids fighting the FPGA-side
 * pull-up if the wiring or pin choice is ever wrong). Resets the CPU/clock
 * domains only -- does NOT reconfigure the FPGA fabric (a bitstream change
 * still needs a real power-cycle). */
static void reset_pulse() {
    pinMode(PIN_RESET, OUTPUT);
    digitalWrite(PIN_RESET, LOW);
    delay(RESET_PULSE_MS);
    pinMode(PIN_RESET, INPUT);
}

/* Boot-mode strap (see PIN_STAY). Open-drain: only ever drive LOW or release to
 * high-Z INPUT, never drive HIGH (never fights the FPGA-side pull-up). The level
 * is held across reset_pulse() so the loader samples it at boot; keep it
 * asserted for the whole flashing session so a mid-flash reset stays in the
 * loader. */
static void stay_assert()  { pinMode(PIN_STAY, OUTPUT); digitalWrite(PIN_STAY, LOW); }
static void stay_release() { pinMode(PIN_STAY, INPUT); }

/* Run one mailbox command; print a labelled line; return the status (or <0). */
static int step(const char *label, uint8_t op, uint32_t a0, uint32_t a1,
                const uint8_t *data, uint32_t n, uint32_t *result) {
    int st = mbx_cmd(op, a0, a1, data, n, result);
    if (st < 0)       Serial.printf("[c3] %-8s LINK ERROR %d\n", label, st);
    else if (st != ST_OK) Serial.printf("[c3] %-8s NAK status 0x%02X\n", label, st);
    return st;
}

static void run_selftest() {
    /* Sanity PING first. */
    uint32_t jedec = 0;
    if (step("PING", CMD_PING, 0, 0, nullptr, 0, &jedec) != ST_OK) return;
    Serial.printf("[c3] PING     ok, JEDEC 0x%06lX\n", (unsigned long)(jedec & 0xFFFFFF));

    /* Build a known 256-byte page + its expected CRCs. */
    uint8_t pat[256], ff[256];
    for (int i = 0; i < 256; i++) { pat[i] = (uint8_t)(i * 7 + 0x11); ff[i] = 0xFF; }
    uint32_t exp_pat = crc32_c3(pat, 256);
    uint32_t exp_ff  = crc32_c3(ff,  256);

    /* Show what's currently in the test sector (informational / safety). */
    uint32_t crc_before = 0;
    step("CRC-pre", CMD_CRC, TEST_OFF, 256, nullptr, 0, &crc_before);
    Serial.printf("[c3] before   CRC 0x%08lX %s\n", (unsigned long)crc_before,
                  crc_before == exp_ff ? "(already blank)" : "(has data -- will be erased)");

    /* 1) ERASE the 4 KB sector. */
    if (step("ERASE", CMD_ERASE, TEST_OFF, 0x1000, nullptr, 0, nullptr) != ST_OK) return;

    /* 2) verify erased -> reads all 0xFF. */
    uint32_t crc_erased = 0;
    step("CRC-er", CMD_CRC, TEST_OFF, 256, nullptr, 0, &crc_erased);
    bool erase_ok = (crc_erased == exp_ff);
    Serial.printf("[c3] erased   CRC 0x%08lX exp 0x%08lX  %s\n",
                  (unsigned long)crc_erased, (unsigned long)exp_ff, erase_ok ? "OK" : "FAIL");

    /* 3) PROGRAM the page. */
    if (step("PROGRAM", CMD_PROGRAM, TEST_OFF, 256, pat, 256, nullptr) != ST_OK) return;

    /* 4) read-back verify: firmware CRCs the flash mmap; compare to expected. */
    uint32_t crc_prog = 0;
    step("CRC-pr", CMD_CRC, TEST_OFF, 256, nullptr, 0, &crc_prog);
    bool prog_ok = (crc_prog == exp_pat);
    Serial.printf("[c3] program  CRC 0x%08lX exp 0x%08lX  %s\n",
                  (unsigned long)crc_prog, (unsigned long)exp_pat, prog_ok ? "OK" : "FAIL");

    Serial.printf("\n[c3] === SELF-TEST %s === (erase+program+verify over SPIBone)\n",
                  (erase_ok && prog_ok) ? "PASS" : "FAIL");
}

/* ---- Host image-streaming protocol (flash_c3.py) over USB-CDC -------------- */
static uint32_t read_exact(uint8_t *buf, uint32_t n, uint32_t timeout_ms) {
    uint32_t got = 0, t0 = millis();
    while (got < n) {
        if (Serial.available()) { buf[got++] = Serial.read(); t0 = millis(); }
        else if (millis() - t0 > timeout_ms) break;
    }
    return got;
}
static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
/* Binary reply: status byte + read-back CRC (LE). NO text on the 'W' path --
 * Serial IS the protocol channel here. */
static void host_reply(uint8_t status, uint32_t crc) {
    uint8_t r[5] = { status, (uint8_t)crc, (uint8_t)(crc >> 8),
                     (uint8_t)(crc >> 16), (uint8_t)(crc >> 24) };
    Serial.write(r, 5);
}

/* 'W' consumed; header = off:u32 LE, len:u32 LE, crc:u32 LE ; then len data bytes
 * sent 256-byte pages at a time, PACED by an ack every WINDOW_PAGES (or at the
 * final partial window) so the C3's USB RX buffer never overflows while it
 * pauses to program (that was the 0xE2 underrun -- see RX_BUFFER_SIZE/
 * WINDOW_PAGES above). Wire sequence:
 *   host -> W, header
 *   C3   -> 1 ack byte after ERASE   (0x01 ok, else 0xE1)
 *   loop: host -> up to WINDOW_PAGES pages ; C3 -> 1 ack byte (0x01 ok, 0xE2 rx, 0xE3 program)
 *   C3   -> final status(1) + read-back crc(4)  (0x00 ok / 0x05 mismatch / 0xE4)
 * NO text anywhere on this path, including on error returns -- flash.py may
 * chain several 'W' commands in one serial session (e.g. bitstream+bios+
 * loader), and any stray byte here gets misread as the NEXT command's ack. */
static void handle_host_flash() {
    uint8_t hdr[12];
    if (read_exact(hdr, 12, 2000) != 12) { Serial.write((uint8_t)0xE0); return; }
    uint32_t off = le32(hdr), len = le32(hdr + 4), exp = le32(hdr + 8);

    /* Erase time scales with image size (per-sector Tse); a flat 5 s timeout is
     * only right for tiny images and falsely flags a large erase as a hang.
     * Budget ~0.5 s per 4 KB sector (worst-case Tse) plus slack so a legitimate
     * multi-second/-minute erase completes. The poll still returns the instant
     * the loader clears MBX_CMD, so a normal erase isn't slowed. */
    uint32_t erase_to = 5000 + (len / 4096 + 1) * 500;
    if (mbx_cmd(CMD_ERASE, off, len, nullptr, 0, nullptr, erase_to) != ST_OK) { Serial.write((uint8_t)0xE1); return; }
    Serial.write((uint8_t)0x01);                 /* erase done -- start sending pages */

    uint8_t page[256];
    uint32_t pages_in_window = 0;
    for (uint32_t pos = 0; pos < len; ) {
        uint32_t n = (len - pos < 256) ? (len - pos) : 256;
        uint32_t got = read_exact(page, n, 5000);
        if (got != n)                            { Serial.write((uint8_t)0xE2); return; }
        if (mbx_cmd(CMD_PROGRAM, off + pos, n, page, n, nullptr) != ST_OK) { Serial.write((uint8_t)0xE3); return; }
        pos += n;
        if (++pages_in_window == WINDOW_PAGES || pos == len) {
            Serial.write((uint8_t)0x01);         /* window programmed -- send next window */
            pages_in_window = 0;
        }
    }

    uint32_t rb = 0;
    if (mbx_cmd(CMD_CRC, off, len, nullptr, 0, &rb) != ST_OK) { host_reply(0xE4, 0); return; }
    host_reply(rb == exp ? 0x00 : 0x05, rb);
}

void loop() {
    if (!Serial.available()) return;
    int c = Serial.read();
    switch (c) {
    case 'W':                                  /* host image stream (binary) */
        handle_host_flash();
        break;
    case 't': case 'T':
        Serial.println("\n[c3] --- running self-test ---");
        run_selftest();
        break;
    case 'p': case 'P': {
        uint32_t j = 0;
        int st = mbx_cmd(CMD_PING, 0, 0, nullptr, 0, &j);
        Serial.printf("[c3] PING st=%d JEDEC 0x%06lX\n", st, (unsigned long)(j & 0xFFFFFF));
        break;
    }
    case 'r': case 'R': {                      /* plain reset (honors current strap) */
        bool via_mbx = mbx_fire(CMD_REBOOT);   /* primary: loader ctrl_reset */
        reset_pulse();                         /* fallback: external line (may be dead) */
        Serial.printf("[c3] reset (%s)\n", via_mbx ? "mailbox ctrl_reset + line" : "line only");
        break;
    }
    case 'l': case 'L': {                      /* enter loader: stay via durable flag */
        /* Primary path: CMD_STAY makes the loader set the boot_ctl flag and
         * ctrl_reset itself -- reliable and independent of the external reset
         * line. The strap + external pulse stay as a belt-and-suspenders
         * fallback (and cover a mid-flash glitch). Needs the loader to already
         * be running to take the mailbox command; from a booted app only the
         * external line/power-cycle can re-enter the loader. */
        stay_assert();
        bool via_mbx = mbx_fire(CMD_STAY);
        reset_pulse();
        Serial.printf("[c3] enter loader: stay asserted, %s (stays resident)\n",
                      via_mbx ? "mailbox CMD_STAY + line" : "line only -- loader not reachable?");
        break;
    }
    case 'b': case 'B': {                      /* boot app: clear stay flag + reset */
        stay_release();
        bool via_mbx = mbx_fire(CMD_BOOT_APP); /* primary: loader clears flag + ctrl_reset */
        /* Let the loader finish CMD_BOOT_APP (clear flag -> ctrl_reset) before
         * the fallback pulse, so a working external line can't preempt the
         * flag-clear and leave a stale stay flag set. Harmless when the line is
         * dead (the current reality) -- reset_pulse() is then a no-op. */
        delay(50);
        reset_pulse();                         /* fallback: external line */
        Serial.printf("[c3] boot app: stay released, %s (chain-boots app)\n",
                      via_mbx ? "mailbox CMD_BOOT_APP + line" : "line only");
        break;
    }
    case 's': case 'S': {                      /* one-shot stay in loader */
        /* End-of-session "stay". Unlike 'l' this RELEASES the strap and relies
         * ONLY on the loader's one-shot boot_ctl flag (CMD_STAY): the loader
         * stays this boot, consumes the flag (stay_requested()), and a later
         * reset chain-boots the app. Holding the strap here (as 'l' does) would
         * make the stay sticky on every reset -- not what --stay wants. */
        stay_release();
        bool via_mbx = mbx_fire(CMD_STAY);
        delay(50);
        reset_pulse();                         /* fallback: external line */
        Serial.printf("[c3] stay (one-shot): strap released, %s "
                      "(loader this boot, app on next reset)\n",
                      via_mbx ? "mailbox CMD_STAY + line" : "line only");
        break;
    }
    default:                                   /* ignore stray bytes / newlines */
        break;
    }
}
