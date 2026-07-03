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

#define SPI_HZ     8000000UL     /* production clock: reliable, ~2x the flash ceiling */
#define POLL_MAX   256

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

/* --- SPIBone Wishbone primitives (byte address; SPIBone shifts internally) --- */
static bool wb_read(uint32_t addr, uint32_t *val) {
    bool ok = false;
    spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
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
    spi.endTransaction();
    return ok;
}

static bool wb_write(uint32_t addr, uint32_t val) {
    spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    cs_lo();
    spi.transfer(0x00);
    spi.transfer((addr >> 24) & 0xFF); spi.transfer((addr >> 16) & 0xFF);
    spi.transfer((addr >>  8) & 0xFF); spi.transfer((addr >>  0) & 0xFF);
    spi.transfer((val >> 24) & 0xFF);  spi.transfer((val >> 16) & 0xFF);
    spi.transfer((val >>  8) & 0xFF);  spi.transfer((val >>  0) & 0xFF);
    bool ok = (wait_marker() == 0x00);
    cs_hi();
    spi.endTransaction();
    return ok;
}

/* Run a mailbox command. Writes arg0/arg1 (+ optional page data), rings the
 * doorbell (cmd), waits for the firmware to clear it, returns status (or <0 on
 * link/timeout error) and *result. `data`/`n` used by PROGRAM (n<=256). */
static int mbx_cmd(uint8_t op, uint32_t arg0, uint32_t arg1,
                   const uint8_t *data, uint32_t n, uint32_t *result) {
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

    uint32_t t0 = millis(), c = 0xffffffff;
    do {
        if (!wb_read(MBX_BASE + MBX_CMD, &c)) return -11;
        if (millis() - t0 > 5000) return -12;             /* firmware never cleared cmd */
    } while (c != 0);

    uint32_t status = 0xff;
    if (!wb_read(MBX_BASE + MBX_STATUS, &status)) return -11;
    if (result && !wb_read(MBX_BASE + MBX_RESULT, result)) return -11;
    return (int)status;
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) ;

    pinMode(PIN_CS, OUTPUT);
    cs_hi();
    spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);

    Serial.println("\n[c3] SPIBone flash loader");
    Serial.printf("[c3] SPI @%lu Hz, mailbox @0x%08lX. cmds: 't'=self-test  'p'=ping"
                  "  'W..'=host image stream (flash_c3.py)\n",
                  (unsigned long)SPI_HZ, (unsigned long)MBX_BASE);
}

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
 * sent one 256-byte page at a time, PACED by a per-page ack so the C3's USB RX
 * buffer never overflows while it pauses to program (that was the 0xE2 underrun).
 * Wire sequence:
 *   host -> W, header
 *   C3   -> 1 ack byte after ERASE   (0x01 ok, else 0xE1)
 *   loop: host -> one page ; C3 -> 1 ack byte (0x01 ok, 0xE2 rx, 0xE3 program)
 *   C3   -> final status(1) + read-back crc(4)  (0x00 ok / 0x05 mismatch / 0xE4) */
static void handle_host_flash() {
    uint8_t hdr[12];
    if (read_exact(hdr, 12, 2000) != 12) { Serial.write((uint8_t)0xE0); return; }
    uint32_t off = le32(hdr), len = le32(hdr + 4), exp = le32(hdr + 8);

    if (mbx_cmd(CMD_ERASE, off, len, nullptr, 0, nullptr) != ST_OK) { Serial.write((uint8_t)0xE1); return; }
    Serial.write((uint8_t)0x01);                 /* erase done -- start sending pages */

    uint8_t page[256];
    for (uint32_t pos = 0; pos < len; ) {
        uint32_t n = (len - pos < 256) ? (len - pos) : 256;
        if (read_exact(page, n, 5000) != n)      { Serial.write((uint8_t)0xE2); return; }
        if (mbx_cmd(CMD_PROGRAM, off + pos, n, page, n, nullptr) != ST_OK) { Serial.write((uint8_t)0xE3); return; }
        pos += n;
        Serial.write((uint8_t)0x01);             /* page programmed -- send next */
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
    default:                                   /* ignore stray bytes / newlines */
        break;
    }
}
