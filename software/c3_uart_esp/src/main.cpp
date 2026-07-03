/* ESP32-C3 UART echo test -- C3 side (transport sanity check).
 *
 * After both SPI transports failed identically ("MISO returns 00") despite the
 * raw-GPIO diag proving every wire good, this checks whether a completely
 * different protocol -- async UART -- works across the same link. The FPGA
 * (icepi_zero_c3uart.py) echoes each received byte in pure gateware; here we send
 * bytes on a hardware UART and verify they come back.
 *
 * Wiring (reuses the MOSI/MISO wires, NO rewiring):
 *   C3 TX = GPIO1 -> FPGA RX (L1/IO24)     C3 RX = GPIO3 <- FPGA TX (M2/IO23)
 *
 * Serial  = USB-CDC console (results).   Serial1 = the FPGA UART link.
 * If echoes match -> the channel + a real protocol work; the bug is SPI-specific.
 */
#include <Arduino.h>

#define PIN_TX   1     /* C3 UART TX -> FPGA RX (L1) */
#define PIN_RX   3     /* C3 UART RX <- FPGA TX (M2) */

#define LINK_BAUD 1200     /* crawl: match the gateware; huge margin vs any RC smear */

static uint32_t pass = 0, fail = 0;

/* Send one byte, wait up to ~20 ms for the echo, return it (or -1 on timeout). */
static int echo_byte(uint8_t b) {
    while (Serial1.available()) Serial1.read();   /* drain stale bytes */
    Serial1.write(b);
    uint32_t t0 = millis();
    while (!Serial1.available())
        if (millis() - t0 > 100)      /* 1200 baud round-trip ~17ms; generous */
            return -1;
    return Serial1.read();
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000)
        ;

    Serial1.begin(LINK_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    Serial.println("\n[c3] UART echo test");
    Serial.printf("[c3] link @ %d 8N1 : C3 TX=GPIO%d->FPGA RX(L1), C3 RX=GPIO%d<-FPGA TX(M2)\n",
                  LINK_BAUD, PIN_TX, PIN_RX);
    Serial.println("[c3] sending bytes, expecting each echoed back...");
}

void loop() {
    /* Walk a few distinctive values so a stuck/short bit is obvious in the echo. */
    static const uint8_t tests[] = { 0x55, 0xAA, 0x01, 0x80, 0x00, 0xFF, 0x5A, 0xC3 };

    for (unsigned i = 0; i < sizeof(tests); i++) {
        int r = echo_byte(tests[i]);
        if (r < 0) {
            Serial.printf("[c3] sent 0x%02X -> TIMEOUT (no echo)\n", tests[i]);
            fail++;
        } else if ((uint8_t)r == tests[i]) {
            Serial.printf("[c3] sent 0x%02X -> echo 0x%02X  OK\n", tests[i], r);
            pass++;
        } else {
            Serial.printf("[c3] sent 0x%02X -> echo 0x%02X  MISMATCH\n", tests[i], r);
            fail++;
        }
    }
    Serial.printf("[c3] --- totals: %lu ok, %lu bad ---\n",
                  (unsigned long)pass, (unsigned long)fail);
    delay(1000);
}
