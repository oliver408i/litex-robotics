/* ESP32-C3 UART RX-report -- C3 side of the receive-side isolation test.
 *
 * The FPGA (icepi_zero_c3uartrx.py) reflects the LAST byte it received on L1
 * back out M2. Here we send a known, changing value on GPIO1 (TX) and check what
 * the FPGA reflects on GPIO3 (RX). Since FPGA->C3 is already proven, whatever
 * comes back tells us the state of the C3->FPGA (L1 receive) path:
 *   reflects our sent value -> FPGA receives on L1 correctly (link is fine).
 *   reflects 0x00 always    -> FPGA receives nothing on L1.
 *   reflects wrong values   -> receiving but mis-framed.
 *
 * Wiring: C3 TX = GPIO1 -> FPGA rx (L1/IO24), C3 RX = GPIO3 <- FPGA tx (M2/IO23).
 */
#include <Arduino.h>

#define PIN_TX   1
#define PIN_RX   3
#define LINK_BAUD 1200

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000)
        ;
    Serial1.begin(LINK_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    Serial.println("\n[c3] UART RX-report test -- probing the C3->FPGA receive path");
    Serial.printf("[c3] send on GPIO%d, read FPGA's reflection on GPIO%d @ %d 8N1\n",
                  PIN_TX, PIN_RX, LINK_BAUD);
}

void loop() {
    static uint8_t tx = 0;
    tx++;                                 /* a fresh, known value each cycle */

    while (Serial1.available()) Serial1.read();   /* clear old reflections */
    Serial1.write(tx);

    delay(300);                           /* let it reach FPGA + reflect back */

    int last = -1;
    while (Serial1.available()) last = Serial1.read();   /* keep the latest */

    if (last < 0)
        Serial.printf("[c3] sent 0x%02X -> nothing reflected (FPGA RX silent)\n", tx);
    else
        Serial.printf("[c3] sent 0x%02X -> FPGA reflects 0x%02X  %s\n", tx, last,
                      (last == tx) ? "MATCH -- FPGA RX works!"
                                   : (last == 0x00 ? "(0x00: FPGA received nothing yet)"
                                                   : "MISMATCH -- receiving but garbled"));
    delay(400);
}
