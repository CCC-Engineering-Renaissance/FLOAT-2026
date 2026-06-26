/*
 * L298N motor test — ESP32-S3 WROOM
 * ------------------------------------------------------------------
 * Drives ONLY the pump via the L298N, no radio/sensor/mission logic, so
 * we can isolate the motor + driver wiring. Cycles: IN -> stop -> OUT ->
 * stop, ramping PWM, printing each step on the UART0 console (115200).
 *
 * WIRING (matches src/main/main.ino):
 *   L298N ENA = GPIO4 (PWM speed)   <- REMOVE the ENA jumper on the board,
 *   L298N IN1 = GPIO5 (direction)      otherwise ENA is tied HIGH and the
 *   L298N IN2 = GPIO6 (direction)      GPIO4 PWM does nothing.
 *   L298N VM  = 12 V battery, logic VCC = 3V3, GND common with the ESP32.
 *   Motor on OUT1/OUT2 (channel A).
 *
 * If the motor still doesn't move: check VM (12 V) present, GND shared with
 * the ESP32, ENA jumper removed, and OUT1/OUT2 actually wired to the pump.
 *
 * BUILD: arduino-cli compile/upload --fqbn esp32:esp32:esp32s3 FLOAT/test/motor_test_s3
 */
#include <Arduino.h>

#define PIN_PUMP_IN1 5   // L298N IN1
#define PIN_PUMP_IN2 6   // L298N IN2
#define PIN_PUMP_PWM 4   // L298N ENA (PWM)
#define PWM_MAX 255

static void drive(int in1, int in2, int pwm) {
  digitalWrite(PIN_PUMP_IN1, in1);
  digitalWrite(PIN_PUMP_IN2, in2);
  analogWrite(PIN_PUMP_PWM, pwm);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PUMP_IN1, OUTPUT);
  pinMode(PIN_PUMP_IN2, OUTPUT);
  pinMode(PIN_PUMP_PWM, OUTPUT);
  drive(LOW, LOW, 0);
  delay(300);
  Serial.println("MOTOR TEST: ENA=GPIO4 IN1=GPIO5 IN2=GPIO6. Cycling IN/OUT...");
}

void loop() {
  Serial.println(">> IN  (IN1=HIGH IN2=LOW), PWM 255 for 3s");
  drive(HIGH, LOW, PWM_MAX);
  delay(3000);

  Serial.println(">> STOP 1s");
  drive(LOW, LOW, 0);
  delay(1000);

  Serial.println(">> OUT (IN1=LOW IN2=HIGH), PWM 255 for 3s");
  drive(LOW, HIGH, PWM_MAX);
  delay(3000);

  Serial.println(">> STOP 1s");
  drive(LOW, LOW, 0);
  delay(1000);

  Serial.println(">> PWM ramp IN: 60,120,180,255 (1s each)");
  for (int p = 60; p <= 255; p += 60) {
    int pwm = (p > 255) ? 255 : p;
    Serial.print("   PWM="); Serial.println(pwm);
    drive(HIGH, LOW, pwm);
    delay(1000);
  }
  drive(LOW, LOW, 0);
  delay(1000);
}
