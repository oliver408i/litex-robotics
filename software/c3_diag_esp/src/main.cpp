/* ESP32-C3 link CONTINUITY DIAGNOSTIC -- C3 side.
 *
 * Companion to software/c3_diag (FPGA side). The C3<->FPGA SPI link has never
 * reliably passed a byte though both sides look healthy. This firmware drops the
 * SPI peripheral and drives the same five wires as raw GPIO so we can prove
 * signal continuity -- and its direction -- wire by wire. No SPI, no protocol.
 *
 * Physical wiring (confirmed 2026-07-01) -- C3 GPIO <-> FPGA ball:
 *   SCLK=GPIO8/P3   MOSI=GPIO1/L1   MISO=GPIO3/M2   CS=GPIO2/J3   READY=GPIO10/R1
 * NOTE: GPIO2 and GPIO8 are ESP32-C3 strapping pins and GPIO9 is BOOT. Driving
 * them AFTER boot (as here) is fine; the FPGA diag gateware deliberately has NO
 * pulls on these balls so nothing drags a strap at C3 reset.
 *
 * Shared 5-bit "raw" value -- bit order MATCHES the FPGA side so the two consoles
 * line up:  bit0 SCLK  bit1 MOSI  bit2 MISO  bit3 CS  bit4 READY.
 *
 * Two modes, toggled by pressing ANY KEY in the monitor:
 *   DRIVE (default): drive all 5 pins with a binary counter (pin i toggles at
 *                    rate 2^i). Run while the FPGA READS -> FPGA console shows a
 *                    clean 0..31 count if every wire is good.
 *   READ           : all 5 pins as inputs; print the level on change + heartbeat.
 *                    Run while the FPGA DRIVES its counter -> proves FPGA->C3.
 *
 * CONTENTION: only one side drives a wire at a time. Defaults are complementary
 * (C3 DRIVE + FPGA READ) so boot-up is safe. Flip BOTH sides together.
 */
#include <Arduino.h>

#define PIN_SCLK   8
#define PIN_MOSI   1
#define PIN_MISO   3
#define PIN_CS     2
#define PIN_READY  10

/* Bit i -> physical pin, in the FPGA's bit order. */
static const uint8_t PIN_OF_BIT[5] = { PIN_SCLK, PIN_MOSI, PIN_MISO, PIN_CS, PIN_READY };
static const char    *NAME_OF_BIT[5] = { "SCLK", "MOSI", "MISO", "CS", "RDY" };

static bool driving = true;         /* start in DRIVE (FPGA starts in READ) */
static uint8_t tick = 0;            /* DRIVE counter */
static uint8_t last_in = 0xff;      /* force first print in READ */
static uint32_t t_step = 0, t_hb = 0;

static void set_mode(bool drive) {
    driving = drive;
    for (int b = 0; b < 5; b++) {
        if (drive) pinMode(PIN_OF_BIT[b], OUTPUT);
        /* INPUT_PULLDOWN, not bare INPUT: an *undriven* wire then reads a solid 0
         * so floating can't masquerade as signal (GPIO2/GPIO8 are strapping pins
         * that float HIGH and would fake SCLK/CS = 1). A real FPGA push-pull drive
         * still overrides the weak pulldown. So in READ: 1 == the FPGA is driving
         * this wire high, guaranteed. */
        else       pinMode(PIN_OF_BIT[b], INPUT_PULLDOWN);
    }
    tick = 0; last_in = 0xff;
    if (drive) Serial.println("\n-> DRIVE mode: driving 2^i counter on all 5. Make sure the FPGA is READING.");
    else       Serial.println("\n-> READ mode: all pins high-Z inputs. Have the FPGA DRIVE now.");
}

static void drive_value(uint8_t v) {
    for (int b = 0; b < 5; b++)
        digitalWrite(PIN_OF_BIT[b], (v >> b) & 1);
}

static uint8_t read_value(void) {
    uint8_t v = 0;
    for (int b = 0; b < 5; b++)
        v |= (digitalRead(PIN_OF_BIT[b]) & 1) << b;
    return v;
}

static void print_state(const char *tag, uint8_t v) {
    Serial.printf("%s raw=0x%02X  SCLK=%d MOSI=%d MISO=%d CS=%d RDY=%d\n",
                  tag, v,
                  (v >> 0) & 1, (v >> 1) & 1, (v >> 2) & 1, (v >> 3) & 1, (v >> 4) & 1);
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000)
        ;                            /* bounded wait for USB-CDC host */

    Serial.println("\n[c3] link continuity DIAGNOSTIC (raw GPIO, no SPI)");
    Serial.printf("[c3] pins: SCLK=GPIO%d MOSI=GPIO%d MISO=GPIO%d CS=GPIO%d READY=GPIO%d\n",
                  PIN_SCLK, PIN_MOSI, PIN_MISO, PIN_CS, PIN_READY);
    Serial.println("[c3] bits: 0=SCLK 1=MOSI 2=MISO 3=CS 4=READY (matches FPGA)");
    Serial.println("[c3] type 'd'=DRIVE, 'r'=READ (repeats OK, sticks). Keep the two sides complementary.");
    set_mode(true);                  /* DRIVE by default */
}

void loop() {
    /* Explicit command chars, not a toggle: a toggle bounces uncontrollably when
     * the console streams the key. 'd'/'r' stick; repeats are idempotent. */
    while (Serial.available()) {
        int c = Serial.read();
        if ((c == 'd' || c == 'D') && !driving) set_mode(true);
        else if ((c == 'r' || c == 'R') && driving) set_mode(false);
    }

    uint32_t now = millis();
    if (driving) {
        if (now - t_step >= 120) {   /* SCLK bit ~4 Hz; full 0..31 cycle ~4 s */
            t_step = now;
            uint8_t v = tick & 0x1F;
            drive_value(v);
            if (v == 0) print_state("[c3] DRIVE wrap ->", 0);
            tick++;
        }
    } else {
        uint8_t v = read_value();
        if (v != last_in) { print_state("[c3] IN change:", v); last_in = v; }
        if (now - t_hb >= 1000) { t_hb = now; print_state("[c3] IN hb:", v); }
    }
}
