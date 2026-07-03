/* ESP32-C3 SPIBone Wishbone-over-SPI master -- C3 side.
 *
 * Reads/writes the FPGA Wishbone bus over LiteX's SPIBone bridge (verified). Now
 * using the C3 HARDWARE SPI peripheral (FSPI) instead of bit-bang, and SWEEPING
 * the SCLK frequency to find the fastest reliable clock. The link + wires are
 * proven good (both transports round-tripped); SPIBone's ceiling is sys/4 =
 * 12.5 MHz, so we probe 1..10 MHz. NOTE the ESP32 rounds SCLK to 80MHz/N, so the
 * actual clock is the nearest divisor at or below the request.
 *
 * SPIBone 4-wire protocol (CPOL0/CPHA0, MSB-first, big-endian, byte-aligned):
 *   READ : MOSI = 0x01, addr[4 BE]; then clock dummies; MISO = 0xFF (busy) until a
 *          0x01 marker byte, then data[4 BE].
 *   WRITE: MOSI = 0x00, addr[4 BE], data[4 BE]; MISO busy 0xFF then a 0x00 marker.
 * SPIBone shifts the byte address internally (bus.adr = address[2:]), so we send
 * the full BYTE address, not a word address.
 *
 * Wiring: SCLK=GPIO8/P3  MOSI=GPIO1/L1  MISO=GPIO3/M2  CS#=GPIO2/J3.
 */
#include <Arduino.h>
#include <SPI.h>

#define PIN_SCLK   8
#define PIN_MOSI   1
#define PIN_MISO   3
#define PIN_CS     2

#define POLL_MAX   256               /* dummy bytes before giving up on a marker */

/* ctrl_scratch BYTE addr (this build: ctrl at page 0). Confirm via csr.csv. */
#define WB_CTRL_SCRATCH 0xF0000004UL

SPIClass spi(FSPI);
static uint32_t g_hz = 1000000;      /* current sweep frequency */

static inline void cs_lo() { digitalWrite(PIN_CS, LOW); }
static inline void cs_hi() { digitalWrite(PIN_CS, HIGH); }

/* Clock dummy 0x00s until MISO leaves 0xFF (busy). Returns the marker byte
 * (0x01 read / 0x00 write) or -1 on timeout. Called inside a transaction. */
static int wait_marker() {
    for (int i = 0; i < POLL_MAX; i++) {
        uint8_t b = spi.transfer(0x00);
        if (b != 0xFF)
            return b;
    }
    return -1;
}

static bool wb_read(uint32_t addr, uint32_t *val) {
    bool ok = false;
    spi.beginTransaction(SPISettings(g_hz, MSBFIRST, SPI_MODE0));
    cs_lo();
    spi.transfer(0x01);
    spi.transfer((addr >> 24) & 0xFF);
    spi.transfer((addr >> 16) & 0xFF);
    spi.transfer((addr >>  8) & 0xFF);
    spi.transfer((addr >>  0) & 0xFF);
    if (wait_marker() == 0x01) {
        uint32_t d = 0;
        for (int i = 0; i < 4; i++)
            d = (d << 8) | spi.transfer(0x00);
        *val = d;
        ok = true;
    }
    cs_hi();
    spi.endTransaction();
    return ok;
}

static bool wb_write(uint32_t addr, uint32_t val) {
    spi.beginTransaction(SPISettings(g_hz, MSBFIRST, SPI_MODE0));
    cs_lo();
    spi.transfer(0x00);
    spi.transfer((addr >> 24) & 0xFF);
    spi.transfer((addr >> 16) & 0xFF);
    spi.transfer((addr >>  8) & 0xFF);
    spi.transfer((addr >>  0) & 0xFF);
    spi.transfer((val  >> 24) & 0xFF);
    spi.transfer((val  >> 16) & 0xFF);
    spi.transfer((val  >>  8) & 0xFF);
    spi.transfer((val  >>  0) & 0xFF);
    bool ok = (wait_marker() == 0x00);
    cs_hi();
    spi.endTransaction();
    return ok;
}

/* Stress one frequency: many write+read-back round-trips over varied bit
 * patterns; count mismatches and timeouts. */
static void test_freq(uint32_t hz, int iters) {
    g_hz = hz;
    static const uint32_t pat[] = {
        0x12345678, 0xDEADBEEF, 0x55555555, 0xAAAAAAAA,
        0xFFFFFFFF, 0x00000000, 0xCAFEBABE, 0x80000001,
    };
    int ok = 0, mism = 0, to = 0;
    for (int i = 0; i < iters; i++) {
        uint32_t v = pat[i % 8] ^ (uint32_t)(i * 0x9E3779B1u);
        uint32_t rb = 0;
        if (!wb_write(WB_CTRL_SCRATCH, v)) { to++; continue; }
        if (!wb_read(WB_CTRL_SCRATCH, &rb)) { to++; continue; }
        if (rb == v) ok++; else mism++;
    }
    Serial.printf("[c3] req %5lu kHz : %3d ok, %3d mismatch, %3d timeout  %s\n",
                  (unsigned long)(hz / 1000), ok, mism, to,
                  (mism == 0 && to == 0) ? "CLEAN" : "<-- errors");
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000)
        ;

    pinMode(PIN_CS, OUTPUT);
    cs_hi();
    spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);   /* manual CS on GPIO2 */

    Serial.println("\n[c3] SPIBone HW-SPI clock sweep (mode 0)");
    Serial.printf("[c3] SCLK=GPIO%d MOSI=GPIO%d MISO=GPIO%d CS=GPIO%d; ceiling sys/4=12.5MHz\n",
                  PIN_SCLK, PIN_MOSI, PIN_MISO, PIN_CS);
    Serial.println("[c3] (ESP32 rounds SCLK to 80MHz/N)");
}

void loop() {
    static const uint32_t freqs[] = {
        1000000, 2000000, 4000000, 8000000, 10000000, 11430000,
    };
    Serial.println("[c3] --- sweep ---");
    for (unsigned i = 0; i < sizeof(freqs)/sizeof(freqs[0]); i++)
        test_freq(freqs[i], 200);
    g_hz = 1000000;
    wb_write(WB_CTRL_SCRATCH, 0x12345678UL);       /* restore scratch reset value */
    Serial.println("[c3] --- done (highest CLEAN row = fastest reliable clock) ---\n");
    delay(3000);
}
