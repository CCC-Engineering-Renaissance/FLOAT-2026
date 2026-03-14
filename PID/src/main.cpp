#include <Arduino.h>
#include <Wire.h> //For Pressure/Depth Sensor
#include "MS5837.h" //For Pressure/Depth Sensor

#define pumpPin1 1 //Pin 1 for Pump Connection
#define pumpPin2 2 //Pin 2 for Pump Connection
#define speedPin 3 //Pin for speed control
MS5837 sensor; //Defines Pressure/Depth Sensor

//Gain constants for Proportional, Integral, and Derivative respectivley. MUST be fine tuned
double kp {1};
double ki {1};
double kd {1};
//test


//Defining Variables
double integral {};
double previousError {};
double targetDepth {2.5};
double error {};
double flowRate {}; //What the PID is changing
double dt {};
double lastTime {0}; //Initialize lastTime. Starts at 0 seconds.


void setup() {

  pinMode(pumpPin1, OUTPUT);
  pinMode(pumpPin2, OUTPUT);
  pinMode(speedPin, OUTPUT);
  Wire.begin();              // Initialize I2C bus
  sensor.init();             // Initialize the sensor
  sensor.setModel(MS5837::MS5837_30BA); //Sets the model of pressure sensor used
  sensor.setFluidDensity(1021.925); //Sets fluid density of EGADS
  
}

// put function declarations here:
double PID(double error);

void loop() {
  //Tracks time passed since start
  double cTime = millis(); //Current time in milliseconds
  dt = (cTime - lastTime)/1000.00; //Change in time Delta t. Divide by 1000 to convert ms to s.
  lastTime = cTime;

  //Measuring depth and returning the error value
  sensor.read(); //Reads the depth it is currently at
  double actualDepth = sensor.depth();
  double error = targetDepth - actualDepth; //Gets error value to plug into PID
  flowRate = PID(error); //Outputs some value. This value will be used to determine whether or not the FLOAT needs to adjust up or down.
  Serial.print("Hello");

  //Correlate flowRate to actual running of pump
  
}

// put function definitions here:
double PID(double error){

double proportional = error;
 integral += error * dt; //Adding because we are taking the sum of the approximation of the area under the curve.
 double derivative = (error - previousError)/dt;
 previousError = error;
 double output = (kp * proportional) + (ki * integral) + (kd * derivative);
 return output;
}
