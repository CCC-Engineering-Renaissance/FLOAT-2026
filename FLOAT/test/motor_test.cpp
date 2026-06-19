/*
 * TB6612FNG Motor / Pump Driver Test  —  Raspberry Pi Pico 2 (RP2350)
 * -------------------------------------------------------------------
 * Standalone sketch to verify the buoyancy-pump wiring on its own,
 * with NONE of the FLOAT mission logic, sensor, or fault handling.
 *
 * WIRING (Updated for hardwired STBY)
 * TB6612 AIN1   -> GP10   (physical pin 14)
 * TB6612 AIN2   -> GP11   (physical pin 15)
 * TB6612 PWMA   -> GP12   (physical pin 16)
 * TB6612 STBY   -> Tie directly to Pico 3V3 (pin 36) or 3.3V power rail <-- Driver always awake
 * TB6612 VM     -> 12 V supply +              <-- the motor will NOT spin without this
 * TB6612 VCC    -> Pico 3V3 (pin 36)
 * TB6612 GND    -> COMMON ground (shared by Pico GND *and* the 12V supply GND)
 * TB6612 A01/A02 -> the pump's two motor leads
 */

const int PIN_AIN1 = 10;   // GP10
const int PIN_AIN2 = 11;   // GP11
const int PIN_PWMA = 12;   // GP12

int  speedPWM = 200;       // 0..255 — Now scales properly to 8-bit max
bool autoMode = true;

// ---- low-level driver helpers ----------------------------------
void motorForward(int pwm) {
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, pwm);
}
void motorReverse(int pwm) {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, HIGH);
  analogWrite(PIN_PWMA, pwm);
}
void motorStop() {
  analogWrite(PIN_PWMA, 0);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
}

// ---- auto-cycle state ------------------------------------------
int           phase = 0;          // 0 fwd, 1 pause, 2 rev, 3 pause
unsigned long phaseStart = 0;
const unsigned long RUN_MS  = 3000;
const unsigned long STOP_MS = 1500;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);

  // Force RP2350 core to use 0-255 range for PWM instead of default 0-1023
  analogWriteRange(255); 
  
  motorStop();

  delay(1500);               // let the Serial Monitor connect
  Serial.println();
  Serial.println(F("=== TB6612 Pump Test (STBY @ 3.3V) ==="));
  Serial.println(F("Auto-cycling. Keys: f r s  + -  a  ?"));
  Serial.print  (F("Speed PWM = ")); Serial.println(speedPWM);
  Serial.println(F("FORWARD 3s..."));

  phase = 0;
  phaseStart = millis();
  motorForward(speedPWM);
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;
    switch (c) {
      case 'f': autoMode = false; motorForward(speedPWM);
                Serial.print(F("FORWARD @ ")); Serial.println(speedPWM); break;
      case 'r': autoMode = false; motorReverse(speedPWM);
                Serial.print(F("REVERSE @ ")); Serial.println(speedPWM); break;
      case 's': autoMode = false; motorStop();
                Serial.println(F("STOP")); break;
      case '+': speedPWM = min(255, speedPWM + 25);
                Serial.print(F("speed = ")); Serial.println(speedPWM); break;
      case '-': speedPWM = max(0,   speedPWM - 25);
                Serial.print(F("speed = ")); Serial.println(speedPWM); break;
      case 'a': autoMode = true; phase = 0; phaseStart = millis();
                motorForward(speedPWM); Serial.println(F("AUTO: FORWARD 3s...")); break;
      case '?': Serial.print(F(autoMode ? "auto" : "manual"));
                Serial.print(F(", speed=")); Serial.println(speedPWM); break;
      default : Serial.print(F("keys: f r s + - a ?  (got '"));
                Serial.print(c); Serial.println(F("')"));
    }
  }
}

void loop() {
  handleSerial();
  if (!autoMode) return;

  unsigned long elapsed = millis() - phaseStart;
  switch (phase) {
    case 0: // forward running
      if (elapsed >= RUN_MS)  { motorStop();            Serial.println(F("stop"));         phase = 1; phaseStart = millis(); }
      break;
    case 1: // pause
      if (elapsed >= STOP_MS) { motorReverse(speedPWM); Serial.println(F("REVERSE 3s...")); phase = 2; phaseStart = millis(); }
      break;
    case 2: // reverse running
      if (elapsed >= RUN_MS)  { motorStop();            Serial.println(F("stop"));         phase = 3; phaseStart = millis(); }
      break;
    case 3: // pause
      if (elapsed >= STOP_MS) { motorForward(speedPWM); Serial.println(F("FORWARD 3s...")); phase = 0; phaseStart = millis(); }
      break;
  }
}
