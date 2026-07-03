/* ESP32-C3 SPI flash-loader bridge -- the "brain" of the post-WINC C3 loader.
 *
 * The FPGA is the SPI SLAVE (gateware/spi_slave.py); this C3 is the master. It
 * drives the dedicated C3 bus and speaks the command protocol implemented in
 * software/c3_loader/main.c (FPGA side). The C3 owns image staging + CRC; the
 * FPGA just programs flash via its LiteSPI master. See docs/c3_loader.md.
 *
 * Current state: bring-up self-test only -- on boot it PINGs the FPGA over SPI
 * and prints the flash JEDEC id over USB-CDC, validating the whole chain
 * (C3 SPI master -> FPGA SPI slave -> firmware -> LiteSPI -> NOR flash) plus the
 * READY handshake. The host<->C3 transfer protocol (stage/CRC/program a real
 * image) is the next layer (step 5).
 *
 * Flow control: the FPGA drives READY low while a flash op runs; we wait for it
 * to return high before reading a reply or sending the next command.
 */
#include <Arduino.h>
#include <SPI.h>

/* ===== C3-side pin map (C3 GPIO numbers) =========================================================
 * The FPGA side is fixed (docs/c3_loader.md):
 *   SCLK=IO27/P3   MOSI=IO24/L1   MISO=IO23/M2   CS#=IO12/J3   READY=IO4/R1
 *
 * GPIO4-7 are the ESP32-C3 JTAG pins (TMS/TDI/TCK/TDO). The USB-JTAG bridge
 * drives GPIO5 (TDI) and GPIO6 (TCK) which fights any software-controlled output
 * and clamps GPIO6 HIGH when idle -- do NOT use GPIO4-7 for SPI.
 */
#define PIN_SCLK    8    /* -> FPGA SCLK (IO27/P3)       was GPIO4 (JTAG TMS) */
#define PIN_MOSI    1    /* -> FPGA MOSI (IO24/L1)       was GPIO6 (JTAG TCK, idles HIGH) */
#define PIN_MISO    3    /* <- FPGA MISO (IO23/M2)       GPIO3, not a JTAG pin */
#define PIN_CS      2    /* -> FPGA CS#  (IO12/J3)       was GPIO7 (JTAG TDO) */
#define PIN_READY   10   /* <- FPGA BUSY/READY (IO4/R1)  GPIO10, not a JTAG pin */

static const uint32_t SPI_HZ = 1000000;   /* 1 MHz for bring-up (FPGA cap ~12 MHz) */

/* Opcodes -- mirror software/c3_loader/main.c */
enum { CMD_NOP = 0x00, CMD_PING = 0x01, CMD_ERASE = 0x02,
       CMD_PROGRAM = 0x03, CMD_CRC = 0x04, CMD_REBOOT = 0x05 };
enum { ST_OK = 0x00 };

#define JEDEC_W25Q128 0xEF4018

SPIClass spi(FSPI);

static inline void cs_lo() { digitalWrite(PIN_CS, LOW); }
static inline void cs_hi() { digitalWrite(PIN_CS, HIGH); }

/* Wait for the FPGA READY line high (accepting / reply ready). */
static bool wait_ready(uint32_t timeout_ms = 10000) {
    uint32_t t0 = millis();
    while (digitalRead(PIN_READY) == LOW)
        if (millis() - t0 > timeout_ms)
            return false;
    return true;
}

/* One CS-framed transfer clocking out `n` bytes of `buf`. */
static void send_frame(const uint8_t *buf, size_t n) {
    spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    cs_lo();
    for (size_t i = 0; i < n; i++)
        spi.transfer(buf[i]);
    cs_hi();
    spi.endTransaction();
}

/* Clock `n` reply bytes in (sending NOPs, which the FPGA ignores). */
static void read_reply(uint8_t *out, size_t n) {
    spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    cs_lo();
    for (size_t i = 0; i < n; i++)
        out[i] = spi.transfer(CMD_NOP);
    cs_hi();
    spi.endTransaction();
}

static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* PING -> flash JEDEC id, or 0 on failure. */
static uint32_t fpga_ping() {
    uint8_t cmd = CMD_PING;
    send_frame(&cmd, 1);
    if (!wait_ready())
        return 0;
    uint8_t r[5];
    read_reply(r, sizeof(r));     /* 0xA5, jedec[4] */
    if (r[0] != 0xA5)
        return 0;
    return le32(&r[1]);
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000)
        ;                          /* wait for USB-CDC host (bounded) */

    /* SLOW OBSERVABLE BIT-BANG: drive every line by hand so CS visibly holds
     * LOW for ~0.8 s per cycle (watch it on a meter) and the FPGA rx-log shows
     * exactly what arrives. No SPI peripheral. */
    pinMode(PIN_SCLK, OUTPUT); digitalWrite(PIN_SCLK, LOW);
    pinMode(PIN_MOSI, OUTPUT); digitalWrite(PIN_MOSI, LOW);
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_CS,   OUTPUT); digitalWrite(PIN_CS,   HIGH);
    pinMode(PIN_READY, INPUT);

    Serial.println("\n[c3] SLOW bit-bang test (clean power)");
    Serial.printf("[c3] CS=GPIO%d SCLK=GPIO%d MOSI=GPIO%d MISO=GPIO%d READY=GPIO%d\n",
                  PIN_CS, PIN_SCLK, PIN_MOSI, PIN_MISO, PIN_READY);
    Serial.println("[c3] watch CS on a meter: it should pulse LOW ~0.8s each cycle");
}

/* Bit-bang a full-duplex byte, MSB-first, mode 0: drive MOSI, sample MISO on the
 * rising edge. Returns the byte read on MISO. */
static const uint32_t BB_MS = 1;   /* ms per half-SCLK (~500 Hz) */

static uint8_t bb_xfer(uint8_t out) {
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--) {
        digitalWrite(PIN_MOSI, (out >> i) & 1);
        delay(BB_MS);
        digitalWrite(PIN_SCLK, HIGH);
        delay(BB_MS);
        in = (in << 1) | (digitalRead(PIN_MISO) & 1);
        digitalWrite(PIN_SCLK, LOW);
    }
    return in;
}

void loop() {
    /* PING round-trip test: frame 1 sends CMD_PING; frame 2 reads the 5-byte
     * reply (0xA5, jedec[4]).  Healthy: "ROUND-TRIP OK (W25Q128)!". */
    digitalWrite(PIN_CS, LOW);
    delay(5);
    bb_xfer(CMD_PING);
    delay(5);
    digitalWrite(PIN_CS, HIGH);

    if (!wait_ready()) {
        Serial.println("[c3] READY timeout waiting for PING reply");
        return;
    }

    uint8_t r[5];
    digitalWrite(PIN_CS, LOW);
    delay(5);
    for (int k = 0; k < 5; k++)
        r[k] = bb_xfer(CMD_NOP);
    digitalWrite(PIN_CS, HIGH);

    uint32_t id = (r[0] == 0xA5) ? le32(&r[1]) : 0;
    Serial.printf("[c3] reply= %02X %02X %02X %02X %02X  %s\n",
                  r[0], r[1], r[2], r[3], r[4],
                  id ? (id == JEDEC_W25Q128 ? "ROUND-TRIP OK (W25Q128)!" : "ok, odd id")
                     : "MISO returned 00");
}
