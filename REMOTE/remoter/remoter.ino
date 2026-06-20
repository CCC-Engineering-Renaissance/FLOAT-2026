#include <HardwareSerial.h>

// HC-12 on UART2 (GPIO 16 & 17)
HardwareSerial HC12(2);

const int HC12_RX = 16;  // RX pin
const int HC12_TX = 17;  // TX pin

void setup() {
  Serial.begin(115200);  // USB serial

  // Initialize HC-12 at 9600 baud
  HC12.begin(9600, SERIAL_8N1, HC12_RX, HC12_TX);

  delay(100);

  // SEND STARTUP SIGNAL
  HC12.print("STARTUP");
  Serial.println("HC-12 Ready - Signal Sent!");
}

void loop() {
  if (HC12.available()) {
    Serial.write(HC12.read());
  }
  if (Serial.available()) {
    HC12.write(Serial.read());
  }
}
