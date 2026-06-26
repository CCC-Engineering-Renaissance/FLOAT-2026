/*
 * Console probe — ESP32-S3. Prints a 1 Hz heartbeat on Serial and nothing
 * else (no I2C, no Serial1). Used to discover which USB path this board's
 * console is actually on:
 *   - built with CDCOnBoot=default -> Serial = UART0 on GPIO43/44 (external
 *     USB-UART bridge boards).
 *   - built with CDCOnBoot=cdc     -> Serial = USB-CDC (native-USB boards).
 * Whichever build makes the heartbeat appear tells us the board topology.
 */
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  static unsigned long t = 0;
  static bool on = false;
  if (millis() - t >= 1000) {
    t = millis();
    on = !on;
    digitalWrite(LED_BUILTIN, on);
    Serial.print("[probe alive] t=");
    Serial.println(millis() / 1000);
  }
}
