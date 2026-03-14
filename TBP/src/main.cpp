#include <Arduino.h>
#include <Wire.h> //For Pressure/Depth Sensor
#include "MS5837.h" //For Pressure/Depth Sensor

#define pumpPin1 1 //Pin 1 for Pump Connection
#define pumpPin2 2 //Pin 2 for Pump Connection
#define speedPin 3 //Pin for speed control
MS5837 sensor; //Defines Pressure/Depth Sensor

void setup() {

  pinMode(pumpPin1, OUTPUT);
  pinMode(pumpPin2, OUTPUT);
  pinMode(speedPin, OUTPUT);
  Wire.begin();              // Initialize I2C bus
  sensor.init();             // Initialize the sensor
  sensor.setModel(MS5837::MS5837_30BA); //Sets the model of pressure sensor used
  sensor.setFluidDensity(1021.925); //Sets fluid density of EGADS
}
/*
  digitalWrite(pumpPin1, HIGH)
  digitalWrite(pumpPin2, LOW)
  Pump spins one way (Don't know which way yet)
*/
/*
  digitalWrite(pumpPin1, LOW)
  digitalWrite(pumpPin2, High)
  Pump spins opposite direction
*/

/*
  digitalWrite(pumpPin1, LOW)
  digitalWrite(pumpPin2, LOW)
  Pump is off
*/
void loop() {
  for (int cycle = 1; cycle < 5; cycle++){
    switch(cycle){
      //Case 1 or Cycle 1 is the first 250cm descent from 0
      case 1:
        analogWrite(speedPin, 255); //Speed of the pump. Value of 0-255 (0 Volts to 5 Volts)
        digitalWrite(pumpPin1, HIGH);
        digitalWrite(pumpPin2, LOW);
        //Pump is spinning one way
        delay(1000); //Keeps the pump running for t amount of time. Ideally long enough to get to desired mass.
        digitalWrite(pumpPin1, LOW); //Pump is now off until we decide to turn it back on

        /*
        Here the float should be descending at a constant rate now. 
        The Float will descend until it begins to approach a desired depth.
        Once it approaches the desired depth, the pump will begin to pump water out to return FLOAT to neutral buoyancy.
        */

        digitalWrite(pumpPin2, HIGH); //Pump is now running in the reverse direction from before. Taking water out, decreasing mass, returning FLOAT to neutral bouyancy.
        delay(1000); //Keeps pump running until FLOAT is at its orignal mass/neutral bouyancy
        digitalWrite(pumpPin2, LOW); //Pump is now off, it's remaining velocity will die off from the drag of the water stopping at 250cm.
        
        if (sensor.depth() == 2.5){
          delay(30000);
        }
        break;
      //Case 2 or Cycle 2 is the first 40cm ascent from 250cm
      case 2:
        digitalWrite(pumpPin2, HIGH); //Pump begins to pump out water to make FLOAT lighter and rise up
        delay(1000); //Keeps pump running until we get to desired mass
        digitalWrite(pumpPin2, LOW); //Pump is off

        /*
        Here the float should be ascending at a constant rate now. 
        The Float will ascend until it begins to approach a desired depth.
        Once it approaches the desired depth, the pump will begin to pump water in to return FLOAT to neutral buoyancy.
        */

        digitalWrite(pumpPin1, HIGH); //Pump is now running in the direction that sucks water in, increasing mass, returning to neutral bouyancy.
        delay(1000); //Keeps pump running until FLOAT is at orignal mass/neutral boyancy
        digitalWrite(pumpPin1, LOW); //Pump is off, remaining velocity will die off from drag, stopping at 40cm.

        if (sensor.depth() == 0.4){
          delay(30000);    
        }
        break;
      //Repeat of Case 1. Times will be different.
      case 3:
        digitalWrite(pumpPin1, HIGH);
        delay(1000);
        digitalWrite(pumpPin1, LOW); 

        /*
        Here the float should be ascending at a constant rate now. 
        The Float will ascend until it begins to approach a desired depth.
        Once it approaches the desired depth, the pump will begin to pump water in to return FLOAT to neutral buoyancy.
        */

        digitalWrite(pumpPin2, HIGH); 
        delay(1000); 
        digitalWrite(pumpPin2, LOW); 

        if (sensor.depth() == 2.5){
          delay(30000);    
        }
        break;
      //Repeat of Case 2. Times will be different.
      case 4:
        digitalWrite(pumpPin2, HIGH);
        delay(1000);
        digitalWrite(pumpPin2, LOW);

        /*
        Here the float should be ascending at a constant rate now. 
        The Float will ascend until it begins to approach a desired depth.
        Once it approaches the desired depth, the pump will begin to pump water in to return FLOAT to neutral buoyancy.
        */

        digitalWrite(pumpPin1, HIGH);
        delay(1000);
        digitalWrite(pumpPin1, LOW);

        if (sensor.depth() == 0.4){
          delay(30000);    
        }
        break;
    }
  }
  digitalWrite(pumpPin1, LOW);
  digitalWrite(pumpPin2, LOW);
  analogWrite(speedPin, 0); //Float is done. Everything should shut off.
}
