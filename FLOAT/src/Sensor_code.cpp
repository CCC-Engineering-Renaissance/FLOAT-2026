#include <HardwareSerial.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <string>
#include "MS5837.h"
#include "PID.cpp"

#define LED 2
#define pumpPin1 1 //Pin 1 for Pump Connection
#define pumpPin2 2 //Pin 2 for Pump Connection
#define speedPin 3 //Pin for speed control


// defines pins
// HC12
#define HC12 Serial1
#define RX_PIN 1
#define TX_PIN 0
#define BAUDRATE 9600
// Motor
int cyclecount = 0;
#define stepPin 5
#define dirPin 18
#define enPin 19
//Goal
float deepDepth = 2.5;
float upperDepth = 0.4;

int packetcount = 0;
unsigned long lastSendTime = 0;
unsigned long sendInterval = 5000; // Send data every 5 seconds

unsigned long holdTime = 30000;
unsigned long holdStart = 0;

string transmitData;

string dataStore;

enum State {
  WAIT,
  DIVE,
  HOLD_DEEP,
  RISE,
  HOLD_upperdepth,
  DONE
};

State currentState = WAIT;

//JSON
struct SensorData {
  int time;
  float pressure;
  float depth;
};

// Function declarations
void sendData(float pressure, float depth, float time);
void PumpIn(int speed);
void PumpOut(int speed);
void PumpStop();
double PID(double error);

void setupSensor() {
  // HC12
  Serial.begin(115200);    // beginning baud rate
  Serial1.begin(BAUDRATE);  // HC-12 module UART initialization

  pinMode(LED, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  pinMode(pumpPin1, OUTPUT);
  pinMode(pumpPin2, OUTPUT);
  pinMode(speedPin, OUTPUT);


  digitalWrite(enPin, HIGH);
  Wire.begin();


  while (!sensor.init()) {
    Serial.println("Sensors Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar2: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(2000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(1021.925);

  Serial.println("waiting for START Command");
}

void loopSensor() {
  //reading sensor 
  sensor.read();
  float depth = sensor.depth();
  float pressure = sensor.pressure();
  unsigned long timeNow = millis() / 1000;
  
switch (currentState) {
      case WAIT:                                // WAIT - Sends data packet before decending
        if (HC12.available()) {
          String command = HC12.readStringUntil('\n');
          command.trim();
          Serial.println("Received command: " + command);
      
        if (command.indexOf("START") != -1) { // Check if the command contains "START"
          Serial.println("Sending Data...");
          sendData(pressure, depth, timeNow); // Send initial data before diving

          currentState = DIVE;                // Transition to DIVE state

          holdStart = millis();
          Serial.println("Received START command. Diving...");
      }
    }
  break;
      case DIVE:                                                          // DIVE - Descends to 2.5m, while adjusting motor speed
        if (depth < deepDepth) {
          double error = deepDepth - depth;                               // Calculate error
          double cTime = millis();                                        //Current time in milliseconds
          dt = (cTime - lastTime)/1000.00;                                //Change in time Delta t. Divide by 1000 to convert ms to s.
          lastTime = cTime;
          
          double flowrate = PID(error);
        if (flowrate > 255) {
            flowrate = 255;
          }
        else if (flowrate < -255) {
            flowrate = -255;
          }
        if (flowrate > 0){
            PumpIn(flowrate);
          } 
        else if (flowrate < 0) {
            PumpOut(abs(flowrate));
          } else {
            PumpStop();
          }
        }
        else {
            PumpStop();
            Serial.println("Reached 2.5m, holding for 30 seconds");
            currentState = HOLD_DEEP;
            holdStart = millis();
            packetcount = 0;
          }
  break;
      case HOLD_DEEP:                                                      // HOLD_DEEP - Holds at 2.5m for 30 seconds, sending data every 5 seconds
        if (millis() - holdStart >= holdTime) {                            // checks to see if 30 seconds have passed and moves to next state
          Serial.println("Hold time at 2.5m complete. Rising...");
          currentState = RISE;
          holdStart = millis();
          packetcount = 0;
     }
         if (millis() - lastSendTime >= sendInterval && packetcount < 7) { // Send data every 5 seconds
          sendData(pressure, depth, timeNow);
          lastSendTime = millis();
          packetcount++;
    }
  break;
      case RISE:                                                         // RISE - Ascends to 0.4m, while adjusting motor speed
        if (depth > upperDepth) {
           double error = upperDepth - depth;                            // Calculate error
           double cTime = millis();                                      //Current time in milliseconds
           dt = (cTime - lastTime)/1000.00;                              //Change in time Delta t. Divide by 1000 to convert ms to s.
           lastTime = cTime;
          
           double flowrate = PID(error);

        if (flowrate > 255) {
           flowrate = 255;
           }
        else if (flowrate < -255) {
            flowrate = -255;
           }
        if (flowrate > 0){
            PumpIn(flowrate);
           } 
        else if (flowrate < 0) {
            PumpOut(abs(flowrate));
           } 
        else {
            PumpStop();
           }
        } 
        else {
            PumpStop();
            Serial.println("Reached 0.4m, holding for 30 seconds");
            currentState = HOLD_upperdepth;
            holdStart = millis();
            packetcount = 0;
          } 
  break;
      case HOLD_upperdepth:                                              // HOLD_upperdepth - Holds at 0.4m for 30 seconds, sending data every 5 seconds
        if (millis() - holdStart >= holdTime) {    
          cyclecount++;                                                  // checks to see if 30 seconds have passed and 
          Serial.println("Hold time at 0.4m complete");
          Serial.println("Cycle count: " + int (cyclecount));
        if (cyclecount < 2) {
          Serial.println("Rising complete. Diving... again");
          currentState = DIVE;
          holdStart = millis();
          packetcount = 0;
        } else {
          Serial.println("Completed final cycle");
          currentState = DONE;
        }
        }
        if (millis() - lastSendTime >= sendInterval && packetcount < 7) { // Send data every 5 seconds
          String trans = String(depth) + " " + String(pressure) + " " + String(time) + "\n";
	  transmitData = transmitData + trans;
          lastSendTime = millis();
          packetcount++;
        }
  break;
      case DONE:
          Serial.println("Completed");
          PumpStop();
	  while (true) {
	  	if (Serial.available()) {
			sendData(transmitData);
		}	
	  }
  break;
    }
  }

void sendData(String data) {
  //String data = "Depth: " + String(depth) + ", Pressure: " + String(pressure) + ", Time: " + int(time);
  HC12.print(data);
}

void PumpIn (int speed) {
  analogWrite(speedPin, speed);
  digitalWrite(pumpPin1, HIGH);
  digitalWrite(pumpPin2, LOW);
}

void PumpOut (int speed) {
  analogWrite(speedPin, speed);
  digitalWrite(pumpPin1, LOW);
  digitalWrite(pumpPin2, HIGH);
}

void PumpStop () {
  analogWrite(speedPin, 0);
  digitalWrite(pumpPin1, LOW);
  digitalWrite(pumpPin2, LOW);
}
