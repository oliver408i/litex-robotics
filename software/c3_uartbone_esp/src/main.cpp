/* ESP32-C3 UARTBone Wishbone-over-UART master -- C3 side.
 *
 * After both SPI transports failed identically ("MISO returns 00") while raw
 * GPIO was perfect, we pivoted to LiteX's UARTBone -- the most battle-tested
 * bridge in the tree (litex_server/wishbone-tool speak it). This is the master
 * half: it reads/writes the FPGA Wishbone bus over a plain hardware UART. No CS,
 * no clock phase, no tristate.
 *
 * Wiring (reuses the MOSI/MISO wires, NO rewiring):
 *   C3 TX = GPIO1 -> FPGA rx (L1/IO24)     C3 RX = GPIO3 <- FPGA tx (M2/IO23)
 * Serial = USB-CDC console (results).   Serial1 = the FPGA UARTBone link.
 *
 * UARTBone protocol (Stream2Wishbone), 8N1, big-endian, WORD addresses:
 *   READ : 0x02, len(words), addr[4 BE];        FPGA returns len*4 data bytes (BE).
 *   WRITE: 0x01, len(words), addr[4 BE], data[4 BE per word];   no reply.
 * The bus is word-addressed, so we send (byte_addr >> 2). ctrl_scratch is byte
 * 0xF0000804 -> word 0x3C000201, reset 0x12345678, R/W: the transport proof.
 */
#include <Arduino.h>

#define PIN_TX   1     /* C3 UART TX -> FPGA rx (L1) */
#define PIN_RX   3     /* C3 UART RX <- FPGA tx (M2) */
#define LINK_BAUD 115200   /* UARTBone's normal range; its 100ms FSM watchdog makes
                              1200 baud unsafe (a single txn is ~75ms on-wire) */

#define CMD_WRITE_BURST_INCR 0x01
#define CMD_READ_BURST_INCR  0x02

/* ctrl_scratch BYTE address. NOTE it moves with the CSR map: this UARTBone build
 * has ctrl at page 0 (0xF0000004); the SPIBone build had an extra CSR at loc 0 that
 * bumped ctrl to page 1 (0xF0000804). Always confirm against build/icepi_zero/csr.csv. */
#define WB_CTRL_SCRATCH 0xF0000004UL

/* Read one 32-bit word at a BYTE address. Returns true + *val, false on timeout. */
static bool wb_read(uint32_t byte_addr, uint32_t *val) {
    uint32_t word = byte_addr >> 2;
    while (Serial1.available()) Serial1.read();     /* drain */

    /* Send the whole command in one write -- no inter-byte gaps that could trip
     * UARTBone's 100ms FSM watchdog. Matches LiteX CommUART: cmd, length,
     * addr//4 big-endian. */
    uint8_t pkt[6] = { CMD_READ_BURST_INCR, 0x01,
                       (uint8_t)(word >> 24), (uint8_t)(word >> 16),
                       (uint8_t)(word >> 8),  (uint8_t)(word >> 0) };
    Serial1.write(pkt, sizeof(pkt));

    uint8_t b[4];
    for (int i = 0; i < 4; i++) {
        uint32_t t0 = millis();
        while (!Serial1.available())
            if (millis() - t0 > 50)
                return false;
        b[i] = Serial1.read();
    }
    *val = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] <<  8) |  (uint32_t)b[3];       /* big-endian */
    return true;
}

/* Write one 32-bit word at a BYTE address. No reply from UARTBone. */
static void wb_write(uint32_t byte_addr, uint32_t val) {
    uint32_t word = byte_addr >> 2;
    uint8_t pkt[10] = { CMD_WRITE_BURST_INCR, 0x01,
                        (uint8_t)(word >> 24), (uint8_t)(word >> 16),
                        (uint8_t)(word >> 8),  (uint8_t)(word >> 0),
                        (uint8_t)(val  >> 24), (uint8_t)(val  >> 16),
                        (uint8_t)(val  >> 8),  (uint8_t)(val  >> 0) };
    Serial1.write(pkt, sizeof(pkt));                 /* contiguous, no gaps */
    Serial1.flush();
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000)
        ;

    Serial1.begin(LINK_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    Serial.println("\n[c3] UARTBone Wishbone master -- transport proof");
    Serial.printf("[c3] link @ %d 8N1 : C3 TX=GPIO%d->FPGA rx(L1), C3 RX=GPIO%d<-FPGA tx(M2)\n",
                  LINK_BAUD, PIN_TX, PIN_RX);
    Serial.println("[c3] target ctrl_scratch byte 0xF0000804 (word 0x3C000201, expect 0x12345678)");
}

void loop() {
    uint32_t v = 0;

    if (!wb_read(WB_CTRL_SCRATCH, &v)) {
        Serial.println("[c3] READ timed out -- no UARTBone reply");
        delay(1000);
        return;
    }
    Serial.printf("[c3] scratch read = 0x%08lX  %s\n", (unsigned long)v,
                  v == 0x12345678UL ? "== 0x12345678 OK" : "(unexpected)");

    const uint32_t test = 0xDEADBEEFUL;
    wb_write(WB_CTRL_SCRATCH, test);
    if (wb_read(WB_CTRL_SCRATCH, &v)) {
        Serial.printf("[c3] wrote 0x%08lX, read back 0x%08lX  %s\n",
                      (unsigned long)test, (unsigned long)v,
                      v == test ? "== ROUND-TRIP OK, UARTBone link proven!" : "MISMATCH");
    }
    wb_write(WB_CTRL_SCRATCH, 0x12345678UL);   /* restore reset value */
    delay(1000);
}
