#include <HardwareSerial.h>

// HC-12 on UART2 (GPIO 16 & 17)
HardwareSerial HC12(2);

const int HC12_RX = 16;  // RX pin
const int HC12_TX = 17;  // TX pin

// Resend the start signal until the float answers, so power-on order between
// the float and this remote doesn't matter (the float takes several seconds to
// boot — sensor init + surface tare — before it can hear us).
const unsigned long STARTUP_RESEND_MS = 1000;  // resend cadence
unsigned long lastStartup = 0;                 // last time we sent "STARTUP"
unsigned long sendCount = 0;                   // how many STARTUP packets sent so far
bool floatResponded = false;                   // true once we hear any reply

void setup() {
  Serial.begin(115200);  // USB serial

  // Initialize HC-12 at 9600 baud
  HC12.begin(9600, SERIAL_8N1, HC12_RX, HC12_TX);

  delay(100);

  Serial.println("HC-12 Ready - sending STARTUP until float responds...");
}

void loop() {
  // Keep nudging the float until it sends something back. Its first packet (or
  // any byte) marks it as awake, after which we stop spamming the channel.
  if (!floatResponded && millis() - lastStartup >= STARTUP_RESEND_MS) {
    HC12.print("STARTUP");
    lastStartup = millis();
    Serial.print("TX -> STARTUP  (#");
    Serial.print(++sendCount);
    Serial.println(")");
  }

  if (HC12.available()) {
    if (!floatResponded) Serial.println(">>> Float responded — HC-12 link confirmed <<<");
    floatResponded = true;          // float is alive — stop resending
    Serial.write(HC12.read());
  }
  if (Serial.available()) {
    HC12.write(Serial.read());
  }
}
