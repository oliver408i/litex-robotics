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

    Serial.println("\n[c3] SPIBone flash loader -- Stage 1 PING");
    Serial.printf("[c3] SPI @%lu Hz, mailbox @0x%08lX\n",
                  (unsigned long)SPI_HZ, (unsigned long)MBX_BASE);
}

void loop() {
    uint32_t jedec = 0;
    int st = mbx_cmd(CMD_PING, 0, 0, nullptr, 0, &jedec);
    if (st < 0) {
        Serial.printf("[c3] PING link error %d (mailbox/SPIBone)\n", st);
    } else if (st != ST_OK) {
        Serial.printf("[c3] PING status 0x%02X (firmware NAK)\n", st);
    } else {
        Serial.printf("[c3] PING ok -- flash JEDEC 0x%06lX  %s\n",
                      (unsigned long)(jedec & 0xFFFFFF),
                      (jedec & 0xFFFFFF) == 0xEF4018 ? "(W25Q128) CHAIN PROVEN!" : "(unexpected id)");
    }
    delay(1000);
}
