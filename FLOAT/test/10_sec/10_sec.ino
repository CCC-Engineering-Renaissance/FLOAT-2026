// =============================================================================
// FLOAT-2026  —  10-SECOND SURFACE TEST  (Raspberry Pi Pico 2 / RP2350)
// -----------------------------------------------------------------------------
// A bench/surface version of the real mission firmware (src/main/main.ino).
// The real firmware dives to 2.5 m and won't stop until depth is reached (or a
// 120 s watchdog trips), which makes it impossible to test on a table. This
// sketch reuses the SAME building blocks — Keller pressure sensor read + surface
// tare, the PID controller, the L298N pump driver, and the exact packet format
// the MainUI_26 GUI expects — but replaces the dive/hold/rise/hold state machine
// with a fixed 10-second collection window:
//
//   power on -> tare -> wait for ANY signal (HC-12 byte, or USB byte in bench
//   mode) -> run PID toward 2.5 m for 10 s while logging ~1 packet/s -> stop the
//   pump -> transmit the buffered packets one at a time, tagged, like the real
//   mission's recovery transmit.
//
// On the surface the depth stays ~0 while the setpoint is 2.5 m, so the PID
// holds a large positive error and the pump runs IN (water in) near full speed
// for the whole window — that is the intended motor-direction test. The 10 s cap
// then stops the pump. If you don't want the pump running dry, unplug pump motor
// power (VM) and just watch the L298N IN1/IN2/ENA signals + sensor + comms.
//
// WIRING (matches src/main/main.ino):
//   HC-12 radio  : Serial1 = UART0 -> GP0 (Pico TX -> HC-12 RX), GP1 (Pico RX <- HC-12 TX)
//   BarXT(Keller): Wire = I2C0 -> GP4 (SDA), GP5 (SCL); power Vin from 5 V rail
//   L298N ch A   : IN1=GP10, IN2=GP11, ENA=GP12 (PWM). Remove the board ENA jumper
//                  if present. IN3/IN4/ENB unused. VM = 12 V, logic VCC = 3V3, GND common.
//   LED          : on-board LED.
//
// BUILD (Pico 2 / RP2350 — use rpipico2, NOT rpipico):
//   arduino-cli compile --fqbn rp2040:rp2040:rpipico2 FLOAT/test/10_sec
//   arduino-cli upload -p <port> --fqbn rp2040:rp2040:rpipico2 FLOAT/test/10_sec
//   (needs the BlueRobotics KellerLD library installed, as in platformio.ini)
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include "KellerLD.h"   // BarXT = Keller 4LD sensor (NOT MS5837)
#include "PID.h"

// -----------------------------------------------------------------------------
// Pin map (same as src/main/main.ino)
// -----------------------------------------------------------------------------
#define PIN_PUMP_IN1  10  // L298N IN1 (direction)
#define PIN_PUMP_IN2  11  // L298N IN2 (direction)
#define PIN_PUMP_PWM  12  // L298N ENA (PWM speed/enable)

#define HC12 Serial1
static const unsigned long HC12_BAUD = 9600;
#define BENCH_TEST   // also accept USB Serial as a trigger; comment out for radio-only
static const unsigned long USB_BAUD  = 115200;

// -----------------------------------------------------------------------------
// Constants (subset of the mission constants, plus the test-specific window)
// -----------------------------------------------------------------------------
#define COMPANY "EX01"                       // packet company-number tag

static const float DEEP_DEPTH    = 2.50f;    // PID setpoint during the test (m)

// Depth is computed from gauge pressure so taring is exact. 997 = fresh water.
static const float FLUID_DENSITY = 997.0f;   // kg/m^3
static const float GRAVITY       = 9.80665f; // m/s^2
static const float SENSOR_MOUNT_OFFSET_M = 0.00f;  // sensor-to-reference offset (m)

static const int   PWM_MAX       = 255;      // pump speed ceiling / PID output limit
static const double KP = 100.0, KI = 5.0, KD = 20.0;  // PLACEHOLDER gains (tune in water)

// Test-specific timing: collect for 10 s, log ~1 packet/s (faster than the
// mission's 5 s cadence) so a short window still yields plenty of packets.
static const unsigned long COLLECT_MS      = 10000;
static const unsigned long PACKET_INTERVAL = 1000;
static const unsigned long TRANSMIT_GAP    = 300;    // spacing between discrete packets

// BOUNDED buffer — fixed array, never an ever-growing String.
static const int MAX_PACKETS = 80;

// -----------------------------------------------------------------------------
// Types & state
// -----------------------------------------------------------------------------
enum State { WAIT, COLLECT, DONE };

struct Packet {
  unsigned long t;   // seconds since START
  float depth;       // m  (tared, mount-offset applied)
  float kpa;         // kPa (gauge)
};

static KellerLD sensor;
static PID      depthPid(KP, KI, KD, -PWM_MAX, PWM_MAX);

static State  state = WAIT;

static Packet packets[MAX_PACKETS];
static int    packetCount   = 0;
static int    transmitIndex = 0;

// surface tare
static float  surfaceMbar = 1013.25f;

// live measurements (refreshed each loop)
static float  g_depth = 0.0f;
static float  g_kpa   = 0.0f;
static bool   g_sensorOk = false;

// timers
static unsigned long missionStart   = 0;
static unsigned long collectStart    = 0;
static unsigned long lastControl     = 0;
static unsigned long packetTimer      = 0;
static unsigned long transmitTimer   = 0;
static unsigned long ledTimer        = 0;

// -----------------------------------------------------------------------------
// Pump (buoyancy engine): one DC motor reversed through an H-bridge.
//   PumpIn  -> water IN  -> heavier -> descend
//   PumpOut -> water OUT -> lighter -> ascend
// -----------------------------------------------------------------------------
static void pumpIn(int speed) {
  if (speed > PWM_MAX) speed = PWM_MAX;
  digitalWrite(PIN_PUMP_IN1, HIGH);
  digitalWrite(PIN_PUMP_IN2, LOW);
  analogWrite(PIN_PUMP_PWM, speed);
}
static void pumpOut(int speed) {
  if (speed > PWM_MAX) speed = PWM_MAX;
  digitalWrite(PIN_PUMP_IN1, LOW);
  digitalWrite(PIN_PUMP_IN2, HIGH);
  analogWrite(PIN_PUMP_PWM, speed);
}
static void pumpStop() {
  analogWrite(PIN_PUMP_PWM, 0);
  digitalWrite(PIN_PUMP_IN1, LOW);
  digitalWrite(PIN_PUMP_IN2, LOW);
}

// -----------------------------------------------------------------------------
// Sensing — reads the BarXT (Keller) and updates g_depth / g_kpa. Returns false
// on an implausible reading. Depth is derived from GAUGE pressure so the surface
// tare makes depth == 0 at the surface, then the mount offset is applied.
// -----------------------------------------------------------------------------
static bool readSensor() {
  sensor.read();
  float absMbar = sensor.pressure();        // mbar absolute (library default)
  if (!(absMbar > 300.0f && absMbar < 20000.0f) || isnan(absMbar)) {
    g_sensorOk = false;
    return false;
  }
  float gaugeMbar = absMbar - surfaceMbar;
  g_kpa   = gaugeMbar / 10.0f;              // 1 mbar = 0.1 kPa
  g_depth = (gaugeMbar * 100.0f) / (FLUID_DENSITY * GRAVITY) + SENSOR_MOUNT_OFFSET_M;
  g_sensorOk = true;
  return true;
}

// Average several samples at the surface on startup to zero pressure.
static void tareSurface() {
  const int N = 20;
  float sum = 0.0f; int good = 0;
  for (int i = 0; i < N; ++i) {
    sensor.read();
    float mb = sensor.pressure();
    if (mb > 300.0f && mb < 20000.0f) { sum += mb; good++; }
    delay(50);
  }
  if (good > 0) surfaceMbar = sum / good;
  Serial.print("Surface tare (mbar): "); Serial.println(surfaceMbar);
  Serial.print("Mount offset (m):   "); Serial.println(SENSOR_MOUNT_OFFSET_M);
}

// -----------------------------------------------------------------------------
// Logging — store into the bounded buffer.
// -----------------------------------------------------------------------------
static void logPacket() {
  if (packetCount >= MAX_PACKETS) return;
  unsigned long tsec = missionStart ? (millis() - missionStart) / 1000 : 0;
  packets[packetCount++] = { tsec, g_depth, g_kpa };
}

// -----------------------------------------------------------------------------
// Packet formatting:  "EX01 0:00:03 9.80 kPa 1.00 m"  (matches src/main/main.ino)
// -----------------------------------------------------------------------------
static String fmtTime(unsigned long sec) {
  unsigned long h = sec / 3600, m = (sec % 3600) / 60, s = sec % 60;
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", h, m, s);
  return String(buf);
}
static String formatPacket(const Packet& p) {
  return String(COMPANY) + " " + fmtTime(p.t) + " " +
         String(p.kpa, 2) + " kPa " + String(p.depth, 2) + " m";
}

// -----------------------------------------------------------------------------
// Transmit: one packet per call, newline-delimited, each tagged with the company
// number. Cycles through the buffer so the shore receiver can catch every packet.
// -----------------------------------------------------------------------------
static void transmitNext() {
  if (packetCount == 0) return;
  unsigned long now = millis();
  if (now - transmitTimer < TRANSMIT_GAP) return;
  transmitTimer = now;
  HC12.println(formatPacket(packets[transmitIndex]));   // '\n' = delimiter
#ifdef BENCH_TEST
  Serial.println(formatPacket(packets[transmitIndex])); // mirror to USB for the bench
#endif
  transmitIndex = (transmitIndex + 1) % packetCount;
}

// -----------------------------------------------------------------------------
// Control: drive the buoyancy engine toward `setpoint`.
//   error > 0 (too shallow)  -> pump IN  (descend)
//   error < 0 (too deep)     -> pump OUT (ascend)
// -----------------------------------------------------------------------------
static void controlDepth(float setpoint) {
  unsigned long now = millis();
  double dt = (now - lastControl) / 1000.0;
  lastControl = now;

  double output = depthPid.compute(setpoint - g_depth, dt);

  if (output > 0.0)      pumpIn((int)output);
  else if (output < 0.0) pumpOut((int)(-output));
  else                   pumpStop();
}

static void enterCollect() {
  state = COLLECT;
  collectStart = millis();
  lastControl  = millis();
  packetTimer  = millis() - PACKET_INTERVAL;  // log immediately on entry
  depthPid.reset();
}

static void heartbeat() {
  static bool ledOn = false;
  unsigned long now = millis();
  unsigned long period = (state == WAIT) ? 1000 : 400;
  if (now - ledTimer >= period) {
    ledTimer = now;
    ledOn = !ledOn;
    digitalWrite(PIN_LED, ledOn ? HIGH : LOW);
  }
}

// =============================================================================
// setup()
// =============================================================================
void setup() {
  Serial.begin(USB_BAUD);
  HC12.begin(HC12_BAUD);            // Serial1 default pins GP0(TX)/GP1(RX)

  pinMode(PIN_PUMP_IN1, OUTPUT);
  pinMode(PIN_PUMP_IN2, OUTPUT);
  pinMode(PIN_PUMP_PWM, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  analogWriteRange(255);            // RP2350 core defaults PWM to 0-1023, not 0-255
  pumpStop();

  Wire.begin();                     // I2C0 GP4(SDA)/GP5(SCL)

  // Bounded init retries — don't spin forever.
  int attempts = 0;
  do {
    sensor.init();
    if (sensor.isInitialized()) break;
    Serial.println("BarXT (Keller) not detected — check I2C wiring and Vin > 3.65 V");
    delay(1000);
  } while (++attempts < 10);
  sensor.setFluidDensity(FLUID_DENSITY);

  tareSurface();                    // zero pressure to atmospheric at surface

  Serial.println("10-SECOND SURFACE TEST ready — send any signal (HC-12 byte, or");
  Serial.println("type a char + Enter here in bench mode) to start the 10 s window.");
}

// =============================================================================
// loop()  —  WAIT -> COLLECT (10 s) -> DONE (transmit)
// =============================================================================
void loop() {
  heartbeat();

  // Refresh measurements every pass (no fault watchdog in this bench test).
  readSensor();

  switch (state) {

    // Start on ANY signal — no "START" keyword required, matching main.ino.
    case WAIT: {
      Stream* in = nullptr;
      if (HC12.available()) in = &HC12;
#ifdef BENCH_TEST
      else if (Serial.available()) in = &Serial;
#endif
      if (in) {
        while (in->available()) in->read();   // drain the trigger bytes
        missionStart = millis();
        Serial.println("Signal received — collecting for 10 s (pump under PID).");
        enterCollect();
      }
      break;
    }

    // Run the real PID toward 2.5 m and log ~1 packet/s for 10 s, then stop.
    case COLLECT: {
      controlDepth(DEEP_DEPTH);

      unsigned long now = millis();
      if (now - packetTimer >= PACKET_INTERVAL) {
        logPacket();
        packetTimer = now;
        if (g_sensorOk) {
          Serial.print("logged  "); Serial.println(formatPacket(packets[packetCount - 1]));
        } else {
          Serial.println("logged  (WARNING: sensor read failed — check wiring)");
        }
      }

      if (now - collectStart >= COLLECT_MS) {
        pumpStop();
        transmitIndex = 0;
        transmitTimer = 0;
        Serial.print("10 s elapsed — "); Serial.print(packetCount);
        Serial.println(" packets buffered. Transmitting.");
        state = DONE;
      }
      break;
    }

    // Transmit the buffered packets discretely, one at a time, tagged.
    case DONE: {
      pumpStop();
      transmitNext();
      break;
    }
  }
}
