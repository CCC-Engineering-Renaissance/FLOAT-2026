/*
 * HC-12 AT-command bridge — Raspberry Pi Pico 2 (RP2350)
 * ------------------------------------------------------------------
 * Transparent USB <-> HC-12 (Serial1) bridge, so you can talk to the
 * float's own HC-12 module directly. Mirrors the passthrough already
 * built into REMOTE/remoter/remoter.ino, for the float side.
 *
 * USE: ground the HC-12's SET pin (puts it into AT-command mode, which
 * always runs at 9600 baud regardless of the module's configured
 * transparent-mode baud), open this board's serial monitor at 115200,
 * type "AT" + Enter -> a working module replies "OK". If it doesn't
 * reply at all, that module's UART/command interface is dead -- a real
 * hardware fault, independent of any wiring/pin/code issue on the host
 * board.
 *
 * Useful commands once you get "OK":
 *   AT+V     -> firmware version (confirms it's a genuine, responding chip)
 *   AT+RX    -> reports current channel/baud/power/mode
 *   AT+Cxxx  -> set channel (e.g. AT+C001)
 *   AT+B9600 -> set baud back to the default used by these sketches
 *
 * Remove the SET-to-GND wire afterward to return to normal transparent
 * mode before re-testing the actual radio link.
 *
 * BUILD:
 *   arduino-cli compile --fqbn rp2040:rp2040:rpipico2 FLOAT/test/hc12_at_bridge
 *   arduino-cli upload -p <port> --fqbn rp2040:rp2040:rpipico2 FLOAT/test/hc12_at_bridge
 */
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);   // AT mode is always 9600, regardless of configured baud
  delay(100);
  Serial.println("HC-12 AT bridge ready. Ground SET, then type AT + Enter.");
}

void loop() {
  if (Serial1.available()) Serial.write(Serial1.read());
  if (Serial.available())  Serial1.write(Serial.read());
}
