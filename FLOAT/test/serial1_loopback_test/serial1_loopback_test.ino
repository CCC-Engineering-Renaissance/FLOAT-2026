/*
 * Serial1 (GP0/GP1) loopback test — Raspberry Pi Pico 2 (RP2350)
 * ------------------------------------------------------------------
 * Bypasses the HC-12 modules entirely, to confirm Serial1 (UART0) is
 * actually listening on GP0 (TX) / GP1 (RX) on THIS board before blaming
 * the HC-12 radio link for the float never receiving anything.
 *
 * WIRING: jumper GP0 directly to GP1. No HC-12, no other hardware needed.
 *
 * BUILD:
 *   arduino-cli compile --fqbn rp2040:rp2040:rpipico2 FLOAT/test/serial1_loopback_test
 *   arduino-cli upload -p <port> --fqbn rp2040:rp2040:rpipico2 FLOAT/test/serial1_loopback_test
 *
 * Expected: "LOOPBACK OK: PING" once a second.
 * If you only ever see "LOOPBACK FAIL", Serial1 isn't on GP0/GP1 as
 * assumed — the HC-12 wiring there would never work regardless of how
 * good the radio link is, and src/main.cpp / hc_12_test.ino's HC12.begin()
 * would need explicit pins.
 */
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  delay(2000);
  Serial.println("Serial1 (GP0 TX / GP1 RX) loopback test - jumper GP0 to GP1 now.");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last >= 1000) {
    last = millis();
    Serial1.print("PING");
    delay(20);  // give the loopback wire + UART time to deliver the bytes
    if (Serial1.available()) {
      Serial.print("LOOPBACK OK: ");
      while (Serial1.available()) Serial.write(Serial1.read());
      Serial.println();
    } else {
      Serial.println("LOOPBACK FAIL: no bytes received");
    }
  }
}
