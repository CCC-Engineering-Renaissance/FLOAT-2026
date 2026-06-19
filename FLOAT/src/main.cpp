// =============================================================================
// FLOAT-2026 firmware  —  MATE "MATE Floats! Under the Ice" vertical profiler
// Single-program firmware for the Raspberry Pi Pico (RP2040), Arduino framework.
//
// This is the ONE program with a single setup()/loop() (REQUIREMENTS A7). It
// folds in what used to live in the uncalled setupSensor()/loopSensor(), drops
// the duplicate setup()/loop() files (transmitter.cpp, old PID.cpp), and uses a
// proper PID class (PID.h / PID.cpp).
//
// Requirement coverage map (see REQUIREMENTS.md):
//   A1  mission sequence (2 profiles, dive/hold/rise/hold, buoyancy only) ... loop()
//   A2  sense + control (depth m, pressure kPa, tare, mount offset, PID) ...... readSensor()/controlDepth()
//   A3  logging (>=5 s cadence, >=20 packets, BOUNDED buffer) ................. logPacket()
//   A4  packet format (company + HH:MM:SS + kPa + m, with units) ............. formatPacket()
//   A5  transmit discretely after recovery, tagged, one-at-a-time ........... transmitNext()
//   A6  safety: on sensor fail / can't hold -> surface + stop pump .......... enterFault()/State FAULT
//   A7  one setup()/loop(), proper PID.h/.cpp, pin conflicts fixed .......... (this file)
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include "KellerLD.h"   // BarXT = Keller 4LD sensor (NOT MS5837)
#include "PID.h"

// -----------------------------------------------------------------------------
// Pin map  (A7: fixes LED 2 vs pumpPin2 2, and pumpPin1 1 vs HC-12 RX 1 / TX 0)
// Hardware: Raspberry Pi Pico 2 (RP2350), Pololu TB6612FNG dual driver (#713),
//           Blue Robotics BarXT (Keller 4LD, I2C 0x40, needs Vin > 3.65 V),
//           HC-12 radio, 12 V peristaltic pump on driver channel A.
//   HC-12 radio : Serial1 = UART0 -> GP0 (Pico TX -> HC-12 RX), GP1 (Pico RX <- HC-12 TX)
//   BarXT (Keller): Wire = I2C0   -> GP4 (SDA), GP5 (SCL); power Vin from 5 V rail
//                   (NOT 3.3 V), I2C pull-ups to 3.3 V.
//   TB6612 ch A : AIN1=GP14, AIN2=GP15, PWMA=GP16, STBY=GP17 (clear of GP0/1 + I2C)
//                 VM = 12 V battery, VCC = Pico 3V3, GND common. STBY must be HIGH.
//   LED         : on-board LED instead of GP2.
// -----------------------------------------------------------------------------
#define PIN_PUMP_IN1  14  // TB6612 AIN1 (direction)
#define PIN_PUMP_IN2  15  // TB6612 AIN2 (direction)
#define PIN_PUMP_PWM  16  // TB6612 PWMA (PWM speed, 0-255)
#define PIN_PUMP_STBY 17  // TB6612 STBY (HIGH = driver enabled; LOW = whole chip off)

#define HC12 Serial1
static const unsigned long HC12_BAUD = 9600;
static const unsigned long USB_BAUD  = 115200;

// -----------------------------------------------------------------------------
// Mission constants
// -----------------------------------------------------------------------------
#define COMPANY "EX01"                       // A4/A5: company number tag

static const float DEEP_DEPTH     = 2.50f;   // A1: descend target (m)
static const float SHALLOW_DEPTH  = 0.40f;   // A1: ascend  target (m)
static const float BAND           = 0.33f;   // A1: ±33 cm hold band
static const float ASCENT_FLOOR   = 0.25f;   // A1: never rise above this (no surface/ice contact)

// A2: depth is computed from gauge pressure so taring is exact. Set the fluid
// density for the competition water: 997 = fresh water (typical pool),
// ~1029 = sea water.
static const float FLUID_DENSITY  = 997.0f;  // kg/m^3
static const float GRAVITY        = 9.80665f;   // m/s^2

// A2: mounting offset between the depth sensor and the rules' reference point
// (bottom of float = 2.5 m, top = 0.40 m). MEASURE THIS on the built float and
// RECORD IT to tell the judge. Positive = reference point is BELOW the sensor.
static const float SENSOR_MOUNT_OFFSET_M = 0.00f;

static const int   PWM_MAX        = 255;     // pump speed ceiling / PID output limit

// A2: PID gains are PLACEHOLDERS — tune in water. (Old code used {1,1,1}.)
static const double KP = 100.0, KI = 5.0, KD = 20.0;

static const unsigned long HOLD_MS         = 30000;   // A1: 30 s in-band hold
static const unsigned long PACKET_INTERVAL = 5000;    // A3: log at least every 5 s
static const unsigned long TRANSMIT_GAP    = 300;     // A5: spacing between discrete packets
static const unsigned long MAX_PHASE_MS    = 120000;  // A6: dive/rise watchdog ("can't hold")
static const unsigned long FAULT_SURFACE_MS= 8000;    // A6: pump-out time to guarantee surfacing
static const int           PROFILES        = 2;       // A1: two full profiles
static const int           FAULT_READ_LIMIT= 10;      // A6: consecutive bad reads -> fault

// A3: BOUNDED buffer — fixed array, never an ever-growing String (heap safety).
static const int MAX_PACKETS = 80;           // 2 profiles need ~ (7+7) hold + transit < 80

// -----------------------------------------------------------------------------
// Types & state
// -----------------------------------------------------------------------------
enum State { WAIT, DIVE, HOLD_DEEP, RISE, HOLD_SHALLOW, DONE, FAULT };

struct Packet {
  unsigned long t;   // seconds since START
  float depth;       // m  (tared, mount-offset applied)
  float kpa;         // kPa (gauge)
};

static KellerLD sensor;
static PID      depthPid(KP, KI, KD, -PWM_MAX, PWM_MAX);

static State  state = WAIT;
static int    cycle = 0;

static Packet packets[MAX_PACKETS];
static int    packetCount    = 0;
static int    transmitIndex  = 0;

// surface tare (A2)
static float  surfaceMbar = 1013.25f;

// live measurements (refreshed each loop)
static float  g_depth = 0.0f;
static float  g_kpa   = 0.0f;
static bool   g_sensorOk = false;
static int    g_readFail = 0;

// timers
static unsigned long missionStart   = 0;
static unsigned long lastControl    = 0;
static unsigned long continuousTimer= 0;
static unsigned long holdPacketTimer= 0;
static unsigned long holdStart      = 0;
static unsigned long phaseStart     = 0;
static unsigned long transmitTimer  = 0;
static unsigned long faultStart     = 0;
static unsigned long ledTimer       = 0;

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
// Sensing (A2)
// -----------------------------------------------------------------------------
// Reads the BarXT (Keller) and updates g_depth / g_kpa. Returns false on an
// implausible reading (used by the A6 fault watchdog). pressure() returns mbar
// by default; depth is derived from GAUGE pressure so the surface tare makes
// depth == 0 at the surface, then the mount offset is applied.
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

// Average several samples at the surface on startup to zero pressure (A2).
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
// Logging (A3) — store into the bounded buffer; oldest data is preserved (we
// simply stop appending once full, which never happens within one mission).
// -----------------------------------------------------------------------------
static void logPacket() {
  if (packetCount >= MAX_PACKETS) return;
  unsigned long tsec = missionStart ? (millis() - missionStart) / 1000 : 0;
  packets[packetCount++] = { tsec, g_depth, g_kpa };
}

// -----------------------------------------------------------------------------
// Packet formatting (A4):  "EX01 1:51:42 9.8 kPa 1.00 m"
// -----------------------------------------------------------------------------
static String fmtTime(unsigned long sec) {
  unsigned long h = sec / 3600, m = (sec % 3600) / 60, s = sec % 60;
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", h, m, s);
  return String(buf);
}
static String formatPacket(const Packet& p) {
  // String(float, 2) is universally available and avoids %f printf-float linker
  // issues across Pico cores.
  return String(COMPANY) + " " + fmtTime(p.t) + " " +
         String(p.kpa, 2) + " kPa " + String(p.depth, 2) + " m";
}

// -----------------------------------------------------------------------------
// Transmit after recovery (A5): one packet per call, newline-delimited, each
// already tagged with the company number. Cycles through the buffer so the
// shore receiver (which filters by company number) can catch every packet even
// while other teams transmit. Replaces the old per-byte blob blast.
// -----------------------------------------------------------------------------
static void transmitNext() {
  if (packetCount == 0) return;
  unsigned long now = millis();
  if (now - transmitTimer < TRANSMIT_GAP) return;
  transmitTimer = now;
  HC12.println(formatPacket(packets[transmitIndex]));   // '\n' = delimiter
  transmitIndex = (transmitIndex + 1) % packetCount;
}

// -----------------------------------------------------------------------------
// Control (A2): drive the buoyancy engine toward `setpoint`.
//   error > 0 (too shallow)  -> pump IN  (descend)
//   error < 0 (too deep)     -> pump OUT (ascend)
// `protectSurface` enforces the A1 ascent floor so the float can't break the
// surface or touch the ice during the shallow phase.
// -----------------------------------------------------------------------------
static void controlDepth(float setpoint, bool protectSurface) {
  unsigned long now = millis();
  double dt = (now - lastControl) / 1000.0;
  lastControl = now;

  double output = depthPid.compute(setpoint - g_depth, dt);

  if (output > 0.0) {
    pumpIn((int)output);
  } else if (output < 0.0) {
    if (protectSurface && g_depth <= ASCENT_FLOOR) pumpStop();  // surface guard
    else pumpOut((int)(-output));
  } else {
    pumpStop();
  }
}

// -----------------------------------------------------------------------------
// Hold helper: count the 30 s only while inside the band; reset the clock the
// instant depth leaves it; log a target packet every 5 s while in band (A1/A3).
// Returns true when a full in-band 30 s hold has completed.
// -----------------------------------------------------------------------------
static bool serviceHold(float setpoint, bool protectSurface) {
  unsigned long now = millis();
  controlDepth(setpoint, protectSurface);

  if (fabs(g_depth - setpoint) <= BAND) {
    if (now - holdPacketTimer >= PACKET_INTERVAL) {
      logPacket();
      holdPacketTimer = now;
    }
    return (now - holdStart >= HOLD_MS);
  }
  // Out of band: reset both the hold clock and the packet cadence so the next
  // in-band entry starts a fresh 0,5,10,...,30 s sequence.
  holdStart = now;
  holdPacketTimer = now - PACKET_INTERVAL;
  return false;
}

// Continuous logging during transit phases (A3).
static void serviceContinuousLog() {
  unsigned long now = millis();
  if (now - continuousTimer >= PACKET_INTERVAL) {
    logPacket();
    continuousTimer = now;
  }
}

// -----------------------------------------------------------------------------
// Fault handling (A6): surface and stop the pump rather than hang.
// -----------------------------------------------------------------------------
static void enterFault(const char* reason) {
  Serial.print("FAULT: "); Serial.println(reason);
  state = FAULT;
  faultStart = millis();
}

static void enterPhase(State s) {
  state = s;
  phaseStart = millis();
  lastControl = millis();
  continuousTimer = millis() - PACKET_INTERVAL;   // log immediately on entry
  holdStart = millis();
  holdPacketTimer = millis() - PACKET_INTERVAL;
  depthPid.reset();
}

static bool isActiveMission(State s) {
  return s == DIVE || s == HOLD_DEEP || s == RISE || s == HOLD_SHALLOW;
}

static void heartbeat() {
  static bool ledOn = false;
  unsigned long now = millis();
  unsigned long period = (state == WAIT) ? 1000 : (state == FAULT ? 150 : 400);
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
  pinMode(PIN_PUMP_STBY, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_PUMP_STBY, HIGH);   // take TB6612 out of standby (required)
  pumpStop();

  Wire.begin();                     // I2C0 GP4(SDA)/GP5(SCL)

  // A6: bounded init retries — don't spin forever. If the sensor never comes
  // up, fall through; the loop's fault path keeps the pump safe.
  // (KellerLD::init() returns void; check isInitialized(). The Keller sensor
  //  reports its own range from PROM, so there is no setModel().)
  int attempts = 0;
  do {
    sensor.init();
    if (sensor.isInitialized()) break;
    Serial.println("BarXT (Keller) not detected — check I2C wiring and Vin > 3.65 V");
    delay(1000);
  } while (++attempts < 10);
  sensor.setFluidDensity(FLUID_DENSITY);

  tareSurface();                    // A2: zero pressure to atmospheric at surface

  Serial.println("FLOAT ready — waiting for START over the radio...");
}

// =============================================================================
// loop()  —  mission state machine (A1)
// =============================================================================
void loop() {
  heartbeat();

  // Refresh measurements every pass; track read health for the A6 watchdog.
  if (readSensor()) {
    g_readFail = 0;
  } else if (++g_readFail >= FAULT_READ_LIMIT && isActiveMission(state)) {
    enterFault("repeated sensor read failures");
  }

  switch (state) {

    // A1: do nothing until a START command arrives over the radio.
    case WAIT: {
      if (HC12.available()) {
        String cmd = HC12.readStringUntil('\n');
        cmd.trim();
        if (cmd.indexOf("START") != -1) {
          missionStart = millis();
          Serial.println("START received — diving.");
          enterPhase(DIVE);
        }
      }
      break;
    }

    // A1: descend to 2.5 m.
    case DIVE: {
      serviceContinuousLog();
      controlDepth(DEEP_DEPTH, false);
      if (fabs(g_depth - DEEP_DEPTH) <= BAND) {
        Serial.println("Reached 2.5 m — holding.");
        holdStart = millis();
        holdPacketTimer = millis() - PACKET_INTERVAL;
        state = HOLD_DEEP;
      } else if (millis() - phaseStart > MAX_PHASE_MS) {
        enterFault("dive timeout — cannot reach 2.5 m");
      }
      break;
    }

    // A1: hold 2.5 m for 30 s (counting in-band time only).
    case HOLD_DEEP: {
      if (serviceHold(DEEP_DEPTH, false)) {
        Serial.println("Deep hold complete — rising.");
        enterPhase(RISE);
      }
      break;
    }

    // A1: ascend to 0.40 m with the surface/ice guard active.
    case RISE: {
      serviceContinuousLog();
      controlDepth(SHALLOW_DEPTH, true);
      if (fabs(g_depth - SHALLOW_DEPTH) <= BAND) {
        Serial.println("Reached 0.40 m — holding.");
        holdStart = millis();
        holdPacketTimer = millis() - PACKET_INTERVAL;
        state = HOLD_SHALLOW;
      } else if (millis() - phaseStart > MAX_PHASE_MS) {
        enterFault("rise timeout — cannot reach 0.40 m");
      }
      break;
    }

    // A1: hold 0.40 m for 30 s, then loop for the next profile or finish.
    case HOLD_SHALLOW: {
      if (serviceHold(SHALLOW_DEPTH, true)) {
        cycle++;
        Serial.print("Profile "); Serial.print(cycle); Serial.println(" complete.");
        if (cycle < PROFILES) {
          enterPhase(DIVE);
        } else {
          pumpStop();
          transmitIndex = 0;
          transmitTimer = 0;
          Serial.print("Mission complete — "); Serial.print(packetCount);
          Serial.println(" packets buffered. Transmitting on recovery.");
          state = DONE;
        }
      }
      break;
    }

    // A5: transmit the buffered packets discretely, one at a time, tagged.
    case DONE: {
      pumpStop();
      transmitNext();
      break;
    }

    // A6: get buoyant (pump out) to surface, then stop the pump and still
    // transmit whatever was logged instead of hanging.
    case FAULT: {
      if (millis() - faultStart < FAULT_SURFACE_MS) {
        pumpOut(PWM_MAX);
      } else {
        pumpStop();
        transmitIndex = 0;
        transmitTimer = 0;
        state = DONE;
      }
      break;
    }
  }
}
