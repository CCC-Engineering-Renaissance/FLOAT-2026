#include <SoftwareSerial.h>
#include "jsonEncode.h"

SoftwareSerial HC12(10,11); //TX,RX

void setup() {
	HC12.begin(9600);
	Serial.begin(9600);
}

void loop() {
	HC12.print("");
}
