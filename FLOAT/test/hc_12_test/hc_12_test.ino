/*
 * HC-12 -> Buoyancy Engine Activation Test  (Raspberry Pi Pico 2 / RP2350)
 * -------------------------------------------------------------------------
 * Bench test for the radio-to-motor activation chain ONLY — none of the
 * FLOAT mission logic, depth sensor, or PID control from src/main.cpp.
 *
 *   ESP32 remote (REMOTE/remoter/remoter.ino) resends "STARTUP" over HC-12
 *   every second until it hears a reply -> this float receives ANY byte on
 *   its HC-12 -> the TB6612 buoyancy pump turns on for TEST_RUN_MS, then
 *   stops on its own and re-arms so the chain can be re-tested by sending
 *   another signal, without reflashing or power-cycling.
 *
 * BUILD: arduino-cli / Arduino IDE sketch (folder name matches this file).
 * IMPORTANT: this is a Pico 2 (RP2350) — use rpipico2, NOT rpipico (RP2040;
 * wrong FQBN flashes fine but the board never boots into USB serial).
 *   arduino-cli compile --fqbn rp2040:rp2040:rpipico2 FLOAT/test/hc_12_test
 *   arduino-cli upload -p <port> --fqbn rp2040:rp2040:rpipico2 FLOAT/test/hc_12_test
 *
 * WIRING (matches src/main.cpp's L298N channel A + HC-12):
 *   HC-12 radio : Serial1 = UART0 -> GP0 (Pico TX -> HC-12 RX), GP1 (Pico RX <- HC-12 TX)
 *   L298N ch A  : IN1=GP10, IN2=GP11, ENA=GP12 (PWM speed/enable). No
 *                 STBY-style chip-sleep pin on the L298N — if the board has an
 *                 ENA jumper, remove it before wiring GP12 there, or the GPIO
 *                 will be fighting a hardwired 5V tie. IN3/IN4/ENB unused.
 *   LED         : on-board LED — slow blink while waiting, solid while the
 *                 pump is running.
 */
#include <Arduino.h>

#define PIN_PUMP_IN1  10  // L298N IN1 (direction)
#define PIN_PUMP_IN2  11  // L298N IN2 (direction)
#define PIN_PUMP_PWM  12  // L298N ENA (PWM speed/enable)

#define HC12 Serial1
static const unsigned long HC12_BAUD = 9600;
#define BENCH_TEST   // also accept USB Serial as a trigger; comment out for radio-only
static const unsigned long USB_BAUD  = 115200;

static const int           TEST_PWM     = 200;   // fixed test speed, 0..255
static const unsigned long TEST_RUN_MS  = 5000;  // auto-stop after this long, then re-arm

static bool          running   = false;
static unsigned long runStart  = 0;

static void pumpOn() {
  digitalWrite(PIN_PUMP_IN1, HIGH);
  digitalWrite(PIN_PUMP_IN2, LOW);
  analogWrite(PIN_PUMP_PWM, TEST_PWM);
}
static void pumpOff() {
  analogWrite(PIN_PUMP_PWM, 0);
  digitalWrite(PIN_PUMP_IN1, LOW);
  digitalWrite(PIN_PUMP_IN2, LOW);
}

void setup() {
  Serial.begin(USB_BAUD);
  HC12.begin(HC12_BAUD);   // Serial1 default pins GP0(TX)/GP1(RX)

  pinMode(PIN_PUMP_IN1, OUTPUT);
  pinMode(PIN_PUMP_IN2, OUTPUT);
  pinMode(PIN_PUMP_PWM, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  analogWriteRange(255);  // RP2350 core defaults PWM to 0-1023, not 0-255
  pumpOff();

  Serial.println("HC-12 activation test ready - waiting for any signal...");
}

void loop() {
  if (running) {
    digitalWrite(PIN_LED, HIGH);
    if (millis() - runStart >= TEST_RUN_MS) {
      pumpOff();
      running = false;
      Serial.println("Test run complete - re-armed, waiting for next signal...");
    }
    return;
  }

  digitalWrite(PIN_LED, (millis() / 500) % 2);   // slow blink while waiting

  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 3000) {
    lastHeartbeat = millis();
    Serial.println("...waiting for HC-12 signal (or type a char here + Enter to trigger manually)");
  }

  bool triggered = false;
  if (HC12.available()) {
    Serial.print("HC12 RX:");
    while (HC12.available()) {
      char c = HC12.read();
      Serial.print(' ');
      Serial.print(c);
    }
    Serial.println();
    triggered = true;
  }
#ifdef BENCH_TEST
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    Serial.println("Manual trigger via USB Serial");
    triggered = true;
  }
#endif

  if (triggered) {
    Serial.println("Signal received -> buoyancy engine ON");
    pumpOn();
    running = true;
    runStart = millis();
  }
}
