/* ESP32-C3 UART listener -- C3 side of the beacon direction-isolation test.
 *
 * The FPGA (icepi_zero_c3uartbeacon.py) continuously transmits an incrementing
 * byte on M2. This listens on GPIO3 (Serial1 RX) and prints what arrives. It's
 * the FPGA->C3 direction ONLY -- the C3 transmits nothing.
 *
 * Wiring: C3 RX = GPIO3 <- FPGA tx (M2/IO23).   Serial = USB-CDC console.
 *
 * Expected if FPGA->C3 works: a clean incrementing stream 00 01 02 03 ...
 *   clean increment  -> FPGA->C3 good, bitstream alive, baud correct.
 *   garbage bytes    -> link/baud wrong (bytes arrive but mis-framed).
 *   nothing at all   -> FPGA not transmitting (dead bitstream) or M2->GPIO3 open.
 */
#include <Arduino.h>

#define PIN_RX   3     /* C3 UART RX <- FPGA tx (M2) */
#define PIN_TX   1     /* unused here, but claim it so it isn't floating */
#define LINK_BAUD 1200

static uint32_t total = 0, in_seq = 0, breaks = 0;
static int last = -1;
static uint32_t t_report = 0;

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000)
        ;
    Serial1.begin(LINK_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    Serial.println("\n[c3] UART LISTENER (beacon test) -- FPGA->C3 only");
    Serial.printf("[c3] listening on GPIO%d @ %d 8N1; expecting incrementing 00,01,02,...\n",
                  PIN_RX, LINK_BAUD);
}

void loop() {
    while (Serial1.available()) {
        int b = Serial1.read();
        total++;
        if (last >= 0) {
            if (b == ((last + 1) & 0xFF)) in_seq++;
            else                          breaks++;
        }
        last = b;
        /* Print the first handful of raw bytes so we can eyeball framing/baud. */
        if (total <= 24)
            Serial.printf("[c3] rx 0x%02X\n", b);
    }

    if (millis() - t_report >= 1000) {
        t_report = millis();
        Serial.printf("[c3] --- rx total=%lu, in-sequence=%lu, breaks=%lu ---\n",
                      (unsigned long)total, (unsigned long)in_seq, (unsigned long)breaks);
    }
}
