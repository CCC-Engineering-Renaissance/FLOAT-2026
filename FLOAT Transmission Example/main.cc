// Example code for HC-12 communication with Arduino Uno
// This code sends "Hello, HC-12!" and listens for incoming data

#include <SoftwareSerial.h>

// Define HC-12 RX and TX pins
SofterwareSerial HC12(10,11); // HC-12 TX pin to Arduino pin 10, RX pin to Arduino pin 11

void setup() {
	Serial.begin(9600);	// Start serial communication with the PC
	HC12.begin(9600);	// Start serial communication with the HC-12 module
	Serial.println("HC-12 Test initialized");
}

void loop() {
	// Send data to HC-12
	HC12.println("Hello, HC-12!");
	delay(1000);

	// Check if data is recieved from HC-12
	if (HC12.available()) {
		String recievedData = HC12.readstring();	// Read incoming data
		Serial.print("Recieved: ");
		Serial.println(recievedData);			// Print recieved data to Serial Monitor
	}
}
