#include <HardwareSerial.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "MS5837.h"


#define LED 2
MS5837 sensor;

float pid(float setpoint, float currentdepth);

// defines pins
// HC-12
#define HC12 Serial1
#define RX_PIN 1
#define TX_PIN 0
#define BAUDRATE 9600
// Motor
int motor_cycles = 0; // remove later
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

float M3_error = 0;
float M3_setpoint = 0;
float M3_previous = 0;
float M3_corrective_val = 0;
int first_run = 0;


void setup() {
  // HC-12
  Serial.begin(115200);                                     // beginning baud rate
  Serial1.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);  // HC-12 module UART initialization


  pinMode(LED, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);


  digitalWrite(enPin, HIGH);
  Wire.begin();


  while (!sensor.init()) {
    Serial.println("Sensors Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar2: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(2000);
  }
  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(997);

  Serial.println("waiting for START Command");
}

void loop() {
  //reading sensor 
  sensor.read();

  float depth = sensor.depth();
  float pressure = sensor.pressure();
  unsigned long timeNow = millis() / 1000;

  

}