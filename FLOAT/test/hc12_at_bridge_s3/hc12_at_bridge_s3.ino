/*
 * HC-12 AT-command bridge — ESP32-S3 WROOM
 * ------------------------------------------------------------------
 * Transparent USB <-> HC-12 (Serial1) bridge so you can talk to the
 * float's own HC-12 module directly and decide if it's fried.
 *
 * No I2C, no sensor, no mission logic — just a radio passthrough plus a
 * 1 Hz "[bridge alive]" heartbeat so the USB console proves itself even
 * if the first prints race USB enumeration.
 *
 * WIRING (matches src/main/main.ino):
 *   HC-12 on Serial1 -> GPIO18 = ESP32 RX (<- HC-12 TX),
 *                       GPIO17 = ESP32 TX (-> HC-12 RX).
 *   (GPIO43/44 are UART0 / the CH343 USB-serial bridge — do NOT use them here.)
 *
 * HOW TO TEST IF THE HC-12 IS FRIED:
 *   1. Ground the HC-12's SET pin (forces AT-command mode @ 9600 baud,
 *      regardless of the configured transparent baud).
 *   2. Open this board's serial monitor at 115200.
 *   3. Type "AT" + Enter. A working module replies "OK".
 *      - "OK"        -> module's command interface is alive.
 *      - no reply    -> with SET grounded and TX/RX correct, the module's
 *                       UART/command path is dead (hardware fault).
 *   Useful follow-ups once you get OK: AT+V (version), AT+RX (settings).
 *   Remove the SET-to-GND wire afterward to return to transparent mode.
 *
 * BUILD (console is UART0 via the CH343 bridge, so CDCOnBoot stays default):
 *   arduino-cli compile --fqbn esp32:esp32:esp32s3 FLOAT/test/hc12_at_bridge_s3
 *   arduino-cli upload  --fqbn esp32:esp32:esp32s3 -p <port> FLOAT/test/hc12_at_bridge_s3
 */
#include <Arduino.h>

#define HC12_RX_PIN 18   // ESP32 RX <- HC-12 TX
#define HC12_TX_PIN 17   // ESP32 TX -> HC-12 RX

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, HC12_RX_PIN, HC12_TX_PIN);
  delay(200);
}

void loop() {
  // Bridge both directions.
  if (Serial1.available()) Serial.write(Serial1.read());
  if (Serial.available())  Serial1.write(Serial.read());

  // Heartbeat so we can confirm the USB console is alive at all.
  static unsigned long t = 0;
  if (millis() - t >= 1000) {
    t = millis();
    Serial.println("[bridge alive] ground HC-12 SET, then type: AT");
  }
}
