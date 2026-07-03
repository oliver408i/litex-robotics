/* ESP32-C3 SPIBone Wishbone-over-SPI master -- C3 side (PHASE 1: transport proof).
 *
 * The raw-GPIO diagnostic proved every C3<->FPGA wire is electrically perfect, so
 * we dropped the custom SPISlave stack and its command protocol in favour of
 * LiteX's maintained SPIBone bridge (icepi_zero_c3spibone.py / gateware
 * add_c3_spibone). This firmware is the master half: it reads/writes the FPGA's
 * Wishbone bus directly. No PING/ERASE/PROGRAM protocol, no READY pin.
 *
 * SPIBone 4-wire protocol (CPOL0/CPHA0, MSB-first, big-endian, byte-aligned):
 *   WRITE: MOSI = 0x00, addr[4 BE], data[4 BE];  MISO = 0xFF (busy) ... then 0x00.
 *   READ : MOSI = 0x01, addr[4 BE], then clock dummies; MISO = 0xFF (busy) until a
 *          0x01 marker byte, then data[4 BE].
 * MISO is driven the whole time CS# is low (idles 0xFF while busy), so we poll by
 * clocking dummy 0x00 bytes until the first non-0xFF byte -- that's the marker.
 * SPIBone runs at <= sys/4 (~12.5 MHz); we use 1 MHz for bring-up.
 *
 * Wiring (confirmed): SCLK=GPIO8/P3  MOSI=GPIO1/L1  MISO=GPIO3/M2  CS#=GPIO2/J3.
 *
 * Transport test: read ctrl_scratch (0xF0000804, reset 0x12345678, R/W), then
 * write 0xDEADBEEF and read it back. If both match -> the verified bridge works
 * end to end. (Confirm the address against build/icepi_zero/csr.csv after build;
 * ctrl is the first CSR so scratch is almost always 0xF0000804.)
 */
#include <Arduino.h>

#define PIN_SCLK   8
#define PIN_MOSI   1
#define PIN_MISO   3
#define PIN_CS     2

/* Bit-banged mode-0 SPI over the GPIO path proven by the raw-GPIO diagnostic --
 * deliberately NOT the FSPI hardware peripheral, so we remove the one unproven
 * link (the C3 SPI block's pin muxing) while bringing up SPIBone. ~2 us/half-bit
 * => ~250 kHz, far under SPIBone's sys/4 (~12.5 MHz) ceiling; SPIBone is edge-
 * synchronous so any slow clock is fine. */
#define BB_US           2
#define POLL_MAX        4096               /* dummy bytes before giving up on a reply */

/* ctrl_scratch BYTE addr for the SPIBone build: ctrl is at page 0 (add_c3_spibone
 * adds only a bus master, no CSR), so scratch = 0xF0000004. (0xF0000804 would hit
 * identifier_mem -> 'i'=0x69.) Confirm per build via build/icepi_zero/csr.csv. */
#define WB_CTRL_SCRATCH 0xF0000004UL

static inline void cs_lo() { digitalWrite(PIN_CS, LOW); }
static inline void cs_hi() { digitalWrite(PIN_CS, HIGH); }

/* One full-duplex byte, MSB-first, mode 0: drive MOSI while SCLK low, sample
 * MISO on the rising edge. */
static uint8_t xfer(uint8_t out) {
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--) {
        digitalWrite(PIN_MOSI, (out >> i) & 1);
        delayMicroseconds(BB_US);
        digitalWrite(PIN_SCLK, HIGH);            /* rising edge: slave samples MOSI */
        in = (in << 1) | (digitalRead(PIN_MISO) & 1);   /* we sample MISO here too */
        delayMicroseconds(BB_US);
        digitalWrite(PIN_SCLK, LOW);
    }
    return in;
}

/* Clock dummy 0x00 bytes until MISO stops reading 0xFF (busy). Returns the first
 * non-0xFF byte (the SPIBone response marker: 0x01 read / 0x00 write), or -1 on
 * timeout. */
static int wait_marker() {
    for (int i = 0; i < POLL_MAX; i++) {
        uint8_t b = xfer(0x00);
        if (b != 0xFF)
            return b;
    }
    return -1;
}

/* Wishbone READ: returns 1 + *val on success, 0 on protocol timeout. */
static bool wb_read(uint32_t addr, uint32_t *val) {
    bool ok = false;
    cs_lo();
    xfer(0x01);                      /* read command */
    xfer((addr >> 24) & 0xFF);       /* address, big-endian */
    xfer((addr >> 16) & 0xFF);
    xfer((addr >>  8) & 0xFF);
    xfer((addr >>  0) & 0xFF);
    int marker = wait_marker();      /* expect 0x01 */
    if (marker == 0x01) {
        uint32_t d = 0;
        for (int i = 0; i < 4; i++)
            d = (d << 8) | xfer(0x00);
        *val = d;
        ok = true;
    }
    cs_hi();
    return ok;
}

/* Wishbone WRITE: returns true once the 0x00 ack marker is seen. */
static bool wb_write(uint32_t addr, uint32_t val) {
    cs_lo();
    xfer(0x00);                      /* write command */
    xfer((addr >> 24) & 0xFF);
    xfer((addr >> 16) & 0xFF);
    xfer((addr >>  8) & 0xFF);
    xfer((addr >>  0) & 0xFF);
    xfer((val  >> 24) & 0xFF);       /* data, big-endian */
    xfer((val  >> 16) & 0xFF);
    xfer((val  >>  8) & 0xFF);
    xfer((val  >>  0) & 0xFF);
    bool ok = (wait_marker() == 0x00);   /* write ack marker is 0x00 */
    cs_hi();
    return ok;
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000)
        ;

    pinMode(PIN_SCLK, OUTPUT); digitalWrite(PIN_SCLK, LOW);   /* mode 0 idle low */
    pinMode(PIN_MOSI, OUTPUT); digitalWrite(PIN_MOSI, LOW);
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_CS,   OUTPUT); digitalWrite(PIN_CS,   HIGH);

    Serial.println("\n[c3] SPIBone Wishbone master -- transport proof (bit-bang)");
    Serial.printf("[c3] SCLK=GPIO%d MOSI=GPIO%d MISO=GPIO%d CS=GPIO%d\n",
                  PIN_SCLK, PIN_MOSI, PIN_MISO, PIN_CS);
    Serial.println("[c3] target ctrl_scratch = 0xF0000804 (expect 0x12345678)");
}

/* RAW DUMP: send a READ command + address, then clock a long run of dummy bytes
 * and print every MISO byte, so we can see exactly what the FPGA returns instead
 * of a binary pass/fail. Interpretation:
 *   all 0xFF          -> MISO driven high the whole time: SPIBone saw no valid
 *                        command, or the bus never ack'd (stuck busy). NOT wiring.
 *   all 0x00 / garbage -> MISO not driven (FPGA never saw CS low / not driving M2).
 *   0xFF..0x01 DE AD BE EF -> it WORKS and only my framing/parse was off.
 */
static void raw_dump(uint32_t addr) {
    const int N = 20;
    uint8_t rx[N];
    cs_lo();
    rx[0] = xfer(0x01);                    /* read command */
    rx[1] = xfer((addr >> 24) & 0xFF);
    rx[2] = xfer((addr >> 16) & 0xFF);
    rx[3] = xfer((addr >>  8) & 0xFF);
    rx[4] = xfer((addr >>  0) & 0xFF);
    for (int i = 5; i < N; i++)            /* dummies: watch MISO for the reply */
        rx[i] = xfer(0x00);
    cs_hi();

    Serial.print("[c3] MISO:");
    for (int i = 0; i < N; i++)
        Serial.printf(" %02X", rx[i]);
    Serial.println();
}

void loop() {
    uint32_t v = 0;

    if (!wb_read(WB_CTRL_SCRATCH, &v)) {
        Serial.println("[c3] READ timed out -- no SPIBone reply (marker never seen)");
        raw_dump(WB_CTRL_SCRATCH);          /* show raw MISO to classify the failure */
        delay(1000);
        return;
    }
    Serial.printf("[c3] scratch read = 0x%08lX  %s\n", (unsigned long)v,
                  v == 0x12345678UL ? "== 0x12345678 OK" : "(unexpected)");

    const uint32_t test = 0xDEADBEEFUL;
    if (wb_write(WB_CTRL_SCRATCH, test) && wb_read(WB_CTRL_SCRATCH, &v))
        Serial.printf("[c3] wrote 0x%08lX, read back 0x%08lX  %s\n",
                      (unsigned long)test, (unsigned long)v,
                      v == test ? "== ROUND-TRIP OK, SPIBone link proven!" : "MISMATCH");
    wb_write(WB_CTRL_SCRATCH, 0x12345678UL);   /* restore reset value */
    delay(1000);
}
