// =============================================================================
// FLOAT-2026 firmware  —  MATE "MATE Floats! Under the Ice" vertical profiler
// ESP32-S3 WROOM, Arduino framework, single setup()/loop().
//
// SENSOR + FALLBACK build. Primary control is CLOSED-LOOP PID on a Keller LD
// (BarXT) pressure sensor read over I2C. If that sensor never initializes (or
// fails mid-run) the firmware degrades gracefully to the OPEN-LOOP dead-reckoning
// estimator below — the mission still runs DIVE -> HOLD -> RISE -> HOLD x2 to
// completion. The sensor is never a hard blocker.
//
//   sensor present & healthy  ->  PID holds MEASURED depth (g_depth), tight band.
//   sensor absent / failed    ->  OPEN-LOOP TIMED schema: pump IN a calibrated
//                                 time to descend, timed 30 s hold, pump OUT a
//                                 calibrated time to ascend, timed hold, x2.
//                                 Sequenced on pump run-time/ballast volume (the
//                                 only reliable observable without a depth sensor),
//                                 NOT on the dead-reckoned depth. CALIBRATE the run
//                                 times (DIVE_RUN_MS / RISE_RUN_MS) in the pool.
//
// HONEST LIMITATIONS (fallback mode only)
//   * The dead-reckoned depth DRIFTS — "hold" is best-effort, not guaranteed
//     +/-33 cm. Calibrate DIVE_TIMEOUT_MS / RISE_TIMEOUT_MS in the pool so phases
//     end even when the estimate drifts.
//   * BUOYANCY BUDGET still applies: at 12 lb in a 2.56 L hull the float is
//     ~2.8 kg negative and physically cannot rise. No code fixes that.
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include "KellerLD.h"   // BarXT = Keller 4LD sensor (NOT MS5837)
#include "PID.h"

// -----------------------------------------------------------------------------
// Pin map for ESP32-S3 WROOM.
//   HC-12 radio : Serial1 -> GPIO17 (ESP32 TX -> HC-12 RX),
//                           GPIO18 (ESP32 RX <- HC-12 TX).
//                 (GPIO43/44 are the UART0 console pins, taken by the onboard
//                  CH343 USB-serial bridge — the HC-12 must NOT use them.)
//   Keller LD   : Wire -> GPIO1 (SDA), GPIO2 (SCL); sensor Vin > 3.65 V.
//   L298N ch A  : IN1=GPIO5, IN2=GPIO6, ENA=GPIO4 (PWM speed/enable).
//                 VM = 12 V battery, logic VCC = 3V3, GND common.
//   LED         : user LED on GPIO21 (cosmetic heartbeat; harmless if absent).
// -----------------------------------------------------------------------------
#define PIN_PUMP_IN1  5    // L298N IN1 (direction)
#define PIN_PUMP_IN2  6    // L298N IN2 (direction)
#define PIN_PUMP_PWM  4    // L298N ENA (PWM speed/enable)
#define PIN_LED       21   // user LED (active-LOW)
#define LED_ON        LOW
#define LED_OFF       HIGH
#define HC12_RX_PIN   18   // ESP32 RX <- HC-12 TX
#define HC12_TX_PIN   17   // ESP32 TX -> HC-12 RX
#define I2C_SDA_PIN   1    // Keller SDA
#define I2C_SCL_PIN   2    // Keller SCL

#define HC12 Serial1
static const unsigned long HC12_BAUD = 9600;
#define BENCH_TEST   // bench testing over USB; comment out for real radio deployment
static const unsigned long USB_BAUD  = 115200;

// -----------------------------------------------------------------------------
// Mission constants
// -----------------------------------------------------------------------------
#define COMPANY "EX01"                       // A4/A5: company number tag

static const float DEEP_DEPTH     = 2.50f;   // A1: descend target (m)
static const float SHALLOW_DEPTH  = 0.40f;   // A1: ascend  target (m)
static const float BAND           = 0.33f;   // A1: ±33 cm hold band
static const float ASCENT_FLOOR   = 0.25f;   // A1: never rise above this (no surface/ice contact)

// Fluid density of the water you actually run in:
//   ~1025 = 2026 MATE Worlds EGADS ice-tank fluid (NRC), SG ~1.025  <- USE FOR WORLDS
//    997  = ordinary fresh-water pool (typical regional)
static const float FLUID_DENSITY  = 1025.0f; // kg/m^3
static const float GRAVITY        = 9.80665f;   // m/s^2

static const int   PWM_MAX        = 255;     // pump speed ceiling / PID output limit
static const double KP = 100.0, KI = 5.0, KD = 20.0;  // PLACEHOLDER gains (A2: tune in water)
static const float SENSOR_MOUNT_OFFSET_M = 0.00f;     // A2: sensor-to-reference offset (m)

// Sensor health: if the sensor was present but reads have been bad this long, fault.
static const unsigned long SENSOR_FAIL_MS = 3000;

// Control source. false = fly the mission on the dead-reckoning METHOD even when a
// pressure sensor is physically present: the sensor still inits and is read (for
// reference/telemetry), but its data NEVER drives control, phase changes, or the
// logged packets. true = closed-loop PID on measured depth (method as fallback).
static const bool CONTROL_USES_SENSOR = false;

// -----------------------------------------------------------------------------
// Physical / hardware measurements (as-built float). The sensorless model and the
// ballast servo run entirely off these — keep them accurate to the real build.
// -----------------------------------------------------------------------------
// Buoyancy engine: 12 V peristaltic pump (positive-displacement) -> internal
// IV-bag reservoir. Measured 0.12919 m/s through a 0.2 in (5.08 mm) bore = 157 mL/min.
static const float PUMP_FLOW_ML_S = 2.62f;     // 157 mL/min, measured
static const float BALLAST_ML     = 200.0f;    // internal IV-bag reservoir (full stroke)
static const unsigned long FULL_STROKE_MS =
    (unsigned long)((BALLAST_ML / PUMP_FLOW_ML_S) * 1000.0f);   // ~76 s end-to-end

// Hull / mass for the dead-reckoning model.
static const float HULL_DISPLACEMENT_ML = 2557.5f;          // outside volume (CAD)
static const float V_HULL_M3            = HULL_DISPLACEMENT_ML * 1e-6f;
static const float FLOAT_MASS_KG        = 5.44f;            // 12 lb, bags empty
static const float HULL_DIAM_M          = 0.11476f;         // largest OD, drag cross-section
static const float A_CROSS              = 0.7853982f * HULL_DIAM_M * HULL_DIAM_M; // PI/4 * D^2
static const float DRAG_CD              = 1.0f;             // bluff cylinder, axial (rough)
static const float ADDED_MASS_CA        = 0.5f;             // entrained-water coefficient

// BUOYANCY BUDGET — buoyancy @1025 = 1025 * 2.5575e-3 = 2.62 kg; net = 2.62 - 5.44 =
//   -2.82 kg. The float is ~2.8 kg NEGATIVE; the 200 mL engine swings only ~+/-0.2 kg.
// WARNING: with this mass/displacement the float CANNOT float or rise — empty bags
//   still sink. The fix is physical: ~5.6 lb mass OR ~5.4 L displacement.

// Per-phase ballast targets (mL). Dive = heaviest, rise = lightest, hold = neutral.
static const float DIVE_BALLAST_ML = BALLAST_ML;   // full -> sink
static const float RISE_BALLAST_ML = 0.0f;         // empty -> rise
static const float BALLAST_TOL_ML  = 2.0f;         // servo deadband

// Phase timeouts. PRIMARY phase end is the depth trigger; these are the fallbacks so
// a phase always ends even when the (fallback) estimate drifts. CALIBRATE in pool.
static const unsigned long DIVE_TIMEOUT_MS = FULL_STROKE_MS + 90000;  // ~166 s
static const unsigned long RISE_TIMEOUT_MS = FULL_STROKE_MS + 90000;  // ~166 s
static const unsigned long HOLD_TIMEOUT_MS = 120000;                  // give up holding -> FAULT

// --- Sensorless (fallback) TIMED schema ---------------------------------------
// Without a depth sensor the ONLY reliable actuator feedback is pump run-time
// (= water volume moved, positive-displacement). So in fallback mode each travel
// phase runs the pump a fixed, calibrated time instead of chasing a depth number.
// CALIBRATE IN THE POOL: time how long the pump must run IN to settle at ~2.5 m
// and OUT to settle at ~0.4 m, then set these. Keep them well under the *_TIMEOUT_MS
// watchdogs above so the schema completes before the safety fault trips.
static const unsigned long DIVE_RUN_MS = 30000;   // pump IN this long -> descend to ~2.5 m
static const unsigned long RISE_RUN_MS = 30000;   // pump OUT this long -> ascend to ~0.4 m

static const unsigned long HOLD_MS         = 30000;   // A1: 30 s in-band hold
static const unsigned long PACKET_INTERVAL = 5000;    // A3: log at least every 5 s
static const unsigned long TRANSMIT_GAP    = 300;     // A5: spacing between discrete packets
static const unsigned long FAULT_SURFACE_MS= (unsigned long)(FULL_STROKE_MS * 1.15f); // ~88 s expel
static const int           PROFILES        = 2;       // A1: two full profiles

static const int MAX_PACKETS = 80;           // bounded buffer (heap safety)

// -----------------------------------------------------------------------------
// Types & state
// -----------------------------------------------------------------------------
enum State { WAIT, DIVE, HOLD_DEEP, RISE, HOLD_SHALLOW, DONE, FAULT };

struct Packet {
  unsigned long t;   // seconds since START
  float depth;       // m  (measured when sensor healthy, else ESTIMATED)
  float kpa;         // kPa
};

static KellerLD sensor;
static PID      depthPid(KP, KI, KD, -PWM_MAX, PWM_MAX);

static State  state = WAIT;
static int    cycle = 0;

static Packet packets[MAX_PACKETS];
static int    packetCount    = 0;
static int    transmitIndex  = 0;

// -----------------------------------------------------------------------------
// Sensor state. surfaceMbar is the startup tare so depth == 0 at the surface.
//   g_sensorPresent : sensor initialized at boot.
//   g_sensorOk      : last read was plausible.
// -----------------------------------------------------------------------------
static float surfaceMbar     = 1013.25f;
static float g_depth         = 0.0f;   // m  (tared, mount-offset applied)
static float g_kpa           = 0.0f;   // kPa (gauge)
static bool  g_sensorOk      = false;
static bool  g_sensorPresent = false;
static unsigned long sensorBadSince = 0; // when reads first went bad (0 = healthy)

// -----------------------------------------------------------------------------
// Sensorless estimator state (dead reckoning) — the fallback.
//   ballast_ml : water in the IV bags, tracked from pump run-time (observable)
//   pump_dir   : +1 pumping IN (heavier), -1 OUT (lighter), 0 stopped
//   est_depth  : ESTIMATED depth (m, >=0). est_vel: ESTIMATED rate (+down).
// -----------------------------------------------------------------------------
static float ballast_ml = 0.0f;   // assume bags start empty at the surface
static int   pump_dir   = 0;
static float est_depth  = 0.0f;
static float est_vel    = 0.0f;
static float est_kpa    = 0.0f;

// timers
static unsigned long missionStart   = 0;
static unsigned long lastModelMs    = 0;
static unsigned long lastControl    = 0;
static unsigned long continuousTimer= 0;
static unsigned long holdPacketTimer= 0;
static unsigned long holdStart      = 0;
static unsigned long phaseStart     = 0;
static unsigned long transmitTimer  = 0;
static unsigned long faultStart     = 0;
static unsigned long ledTimer       = 0;

// -----------------------------------------------------------------------------
// Mode select: which depth source the state machine acts on this pass.
//   useSensor() -> true once the sensor inited AND is returning plausible reads.
//   depthNow()  -> measured depth when healthy, else dead-reckoned estimate.
// -----------------------------------------------------------------------------
// CONTROL_USES_SENSOR == false forces the METHOD: useSensor() is always false, so
// depthNow() and every control/phase/log decision uses the dead-reckoned estimate
// regardless of whether the Keller LD is present and healthy.
static bool  useSensor() { return CONTROL_USES_SENSOR && g_sensorPresent && g_sensorOk; }
static float depthNow()  { return useSensor() ? g_depth : est_depth; }

static bool isActiveMission(State s);   // fwd decl: updateModel() gates on it

// -----------------------------------------------------------------------------
// Pump (buoyancy engine). Each call also sets pump_dir so the ballast integrator
// knows which way water is moving.
//   pumpIn  -> water IN  -> heavier -> descend
//   pumpOut -> water OUT -> lighter -> ascend
// -----------------------------------------------------------------------------
static void pumpIn(int speed) {
  if (speed > PWM_MAX) speed = PWM_MAX;
  digitalWrite(PIN_PUMP_IN1, HIGH);
  digitalWrite(PIN_PUMP_IN2, LOW);
  analogWrite(PIN_PUMP_PWM, speed);
  pump_dir = (speed > 0) ? +1 : 0;
}
static void pumpOut(int speed) {
  if (speed > PWM_MAX) speed = PWM_MAX;
  digitalWrite(PIN_PUMP_IN1, LOW);
  digitalWrite(PIN_PUMP_IN2, HIGH);
  analogWrite(PIN_PUMP_PWM, speed);
  pump_dir = (speed > 0) ? -1 : 0;
}
static void pumpStop() {
  analogWrite(PIN_PUMP_PWM, 0);
  digitalWrite(PIN_PUMP_IN1, LOW);
  digitalWrite(PIN_PUMP_IN2, LOW);
  pump_dir = 0;
}

// -----------------------------------------------------------------------------
// Ballast servo (fallback control): drive `ballast_ml` toward a target volume by
// running the pump. Positive-displacement, so run-time IS volume — reliable even
// without a depth sensor. Returns true when within the deadband.
// -----------------------------------------------------------------------------
static bool servoBallast(float target_ml) {
  if (ballast_ml < target_ml - BALLAST_TOL_ML)      { pumpIn(PWM_MAX);  return false; }
  else if (ballast_ml > target_ml + BALLAST_TOL_ML) { pumpOut(PWM_MAX); return false; }
  else                                              { pumpStop();       return true;  }
}

// Neutral-trim ballast (mL): mass + rho*V_ball = rho*V_hull  ->  V_ball = V_hull - m/rho.
// For the as-built float this is negative, so it clamps to 0 (still sinks — see budget).
static float neutralBallastMl() {
  float v = (V_HULL_M3 - FLOAT_MASS_KG / FLUID_DENSITY) * 1e6f;
  if (v < 0.0f)        v = 0.0f;
  if (v > BALLAST_ML)  v = BALLAST_ML;
  return v;
}

// -----------------------------------------------------------------------------
// Closed-loop control (sensor mode): drive the buoyancy engine toward `setpoint`
// using the PID on MEASURED depth.
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

// -----------------------------------------------------------------------------
// Sensing — reads the Keller LD and updates g_depth / g_kpa. Returns false on an
// implausible reading. Depth is derived from GAUGE pressure so the surface tare
// makes depth == 0 at the surface, then the mount offset is applied.
// -----------------------------------------------------------------------------
static bool readSensor() {
  if (!g_sensorPresent) { g_sensorOk = false; return false; }
  sensor.read();
  float absMbar = sensor.pressure();        // mbar absolute (library default)
  if (isnan(absMbar) || !(absMbar > 300.0f && absMbar < 20000.0f)) {
    g_sensorOk = false;
    return false;
  }
  float gaugeMbar = absMbar - surfaceMbar;
  g_kpa   = gaugeMbar / 10.0f;              // 1 mbar = 0.1 kPa
  g_depth = (gaugeMbar * 100.0f) / (FLUID_DENSITY * GRAVITY) + SENSOR_MOUNT_OFFSET_M;
  g_sensorOk = true;
  return true;
}

// Average several samples at the surface on startup to zero pressure (A2 tare).
static void tareSurface() {
  if (!g_sensorPresent) {
    Serial.println("Surface tare skipped: Keller LD not initialized (fallback mode).");
    return;
  }
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

static void scanI2C() {
  Serial.print("I2C scan on Wire - SDA=GPIO");
  Serial.print(I2C_SDA_PIN);
  Serial.print(" SCL=GPIO");
  Serial.print(I2C_SCL_PIN);
  Serial.println(":");

  int found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("  found device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      found++;
    }
  }
  if (found == 0) Serial.println("  no I2C devices found");
  Serial.print("I2C scan done, ");
  Serial.print(found);
  Serial.println(" device(s).");
}

// -----------------------------------------------------------------------------
// Sensorless model update (dead reckoning) — the fallback estimator. Integrates
// ballast from pump run-time, then integrates the vertical equation of motion.
//   F_down = (m_dry + rho*V_ballast)*g  -  rho*g*V_hull      (weight - buoyancy)
//   F_drag = 0.5 * rho * Cd * A * v*|v|                      (opposes motion)
//   a      = (F_down - F_drag) / (m + Ca*rho*V_hull)         (added mass)
// -----------------------------------------------------------------------------
static void updateModel() {
  unsigned long now = millis();
  float dt = (now - lastModelMs) / 1000.0f;
  lastModelMs = now;
  if (dt <= 0.0f) return;
  if (dt > 0.5f)  dt = 0.5f;                 // clamp after a stall so we don't jump

  // 1) Integrate ballast volume from pump direction (positive-displacement).
  ballast_ml += pump_dir * PUMP_FLOW_ML_S * dt;
  if (ballast_ml < 0.0f)        ballast_ml = 0.0f;
  if (ballast_ml > BALLAST_ML)  ballast_ml = BALLAST_ML;

  // 2) Vertical dynamics — ONLY while actually running a mission phase. Integrating
  //    while WAITING at the surface made the estimate walk to metres before launch
  //    and collapse the DIVE trigger; freeze it until the mission starts.
  if (isActiveMission(state)) {
    float V_ball = ballast_ml * 1e-6f;
    float mass   = FLOAT_MASS_KG + FLUID_DENSITY * V_ball;
    float F_down = mass * GRAVITY - FLUID_DENSITY * GRAVITY * V_HULL_M3;
    float F_drag = 0.5f * FLUID_DENSITY * DRAG_CD * A_CROSS * est_vel * fabsf(est_vel);
    float m_eff  = mass + ADDED_MASS_CA * FLUID_DENSITY * V_HULL_M3;
    float a      = (F_down - F_drag) / m_eff;

    est_vel   += a * dt;
    est_depth += est_vel * dt;
    if (est_depth < 0.0f) {                   // hit the surface
      est_depth = 0.0f;
      if (est_vel < 0.0f) est_vel = 0.0f;
    }
  } else {
    est_vel = 0.0f;                           // not under way (WAIT / DONE / FAULT)
  }

  // Estimated gauge pressure from estimated depth (P = rho*g*h), kPa.
  est_kpa = FLUID_DENSITY * GRAVITY * est_depth / 1000.0f;
}

// -----------------------------------------------------------------------------
// Logging (A3) — bounded buffer. Uses measured depth/pressure when the sensor is
// healthy, else the dead-reckoned estimate.
// -----------------------------------------------------------------------------
static void logPacket() {
  if (packetCount >= MAX_PACKETS) return;
  unsigned long tsec = missionStart ? (millis() - missionStart) / 1000 : 0;
  float d = useSensor() ? g_depth : est_depth;
  float p = useSensor() ? g_kpa   : est_kpa;
  packets[packetCount++] = { tsec, d, p };
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
  return String(COMPANY) + " " + fmtTime(p.t) + " " +
         String(p.kpa, 2) + " kPa " + String(p.depth, 2) + " m";
}

// -----------------------------------------------------------------------------
// Transmit after recovery (A5): one packet per call, newline-delimited, tagged.
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
// Hold helper: servo to neutral and count 30 s only while depthNow() is in band;
// reset the clock the instant it leaves. Logs a packet every 5 s while in band.
// `protectSurface` adds ballast if the depth rises above the ascent floor.
// Returns true on a completed in-band 30 s hold.
// -----------------------------------------------------------------------------
static bool serviceHold(float setpoint, bool protectSurface) {
  unsigned long now = millis();
  float depth = depthNow();

  if (protectSurface && depth <= ASCENT_FLOOR) {
    pumpIn(PWM_MAX);                       // too shallow: add ballast, sink back
  } else if (useSensor()) {
    controlDepth(setpoint);                // closed-loop hold on measured depth
  } else {
    servoBallast(neutralBallastMl());      // fallback: best-effort neutral trim
  }

  if (fabsf(depth - setpoint) <= BAND) {
    if (now - holdPacketTimer >= PACKET_INTERVAL) {
      logPacket();
      holdPacketTimer = now;
    }
    return (now - holdStart >= HOLD_MS);
  }
  holdStart = now;                          // out of band: reset hold + packet cadence
  holdPacketTimer = now - PACKET_INTERVAL;
  return false;
}

static void serviceContinuousLog() {
  unsigned long now = millis();
  if (now - continuousTimer >= PACKET_INTERVAL) {
    logPacket();
    continuousTimer = now;
  }
}

// -----------------------------------------------------------------------------
// Sensorless (fallback) hold: there is no depth to servo on, so just FREEZE the
// ballast where the travel phase left it (full = stay deep, empty = stay shallow)
// and count a fixed 30 s, logging a packet every 5 s (A3: 7 packets per hold).
// Deterministic — always completes in HOLD_MS, so it never falls through to the
// hold watchdog/FAULT the way the depth-band hold does without a sensor.
// -----------------------------------------------------------------------------
static bool serviceTimedHold() {
  pumpStop();                                  // hold current ballast, no depth feedback
  unsigned long now = millis();
  if (now - holdPacketTimer >= PACKET_INTERVAL) {
    logPacket();
    holdPacketTimer = now;
  }
  return (now - holdStart >= HOLD_MS);
}

// -----------------------------------------------------------------------------
// Fault handling (A6): surface (expel all ballast) and stop, rather than hang.
// -----------------------------------------------------------------------------
static void enterFault(const char* reason) {
  Serial.print("FAULT: "); Serial.println(reason);
  state = FAULT;
  faultStart = millis();
}

static void enterPhase(State s) {
  state = s;
  phaseStart = millis();
  continuousTimer = millis() - PACKET_INTERVAL;   // log immediately on entry
  holdStart = millis();
  holdPacketTimer = millis() - PACKET_INTERVAL;
  lastControl = millis();
  depthPid.reset();                               // fresh hold: no inherited integral
}

static bool isActiveMission(State s) {
  return s == DIVE || s == HOLD_DEEP || s == RISE || s == HOLD_SHALLOW;
}

// Watch a sensor that was present at boot but starts returning bad reads mid-run.
// A sensor that was NEVER present is not a fault — that's the fallback path.
static void checkSensorHealth() {
  if (!CONTROL_USES_SENSOR) return;   // sensor ignored for control -> its health can't fault the run
  if (!g_sensorPresent) return;
  if (g_sensorOk) { sensorBadSince = 0; return; }
  unsigned long now = millis();
  if (sensorBadSince == 0) sensorBadSince = now;
  else if (now - sensorBadSince >= SENSOR_FAIL_MS && isActiveMission(state)) {
    enterFault("sensor read failure");
  }
}

static void heartbeat() {
  static bool ledOn = false;
  unsigned long now = millis();
  unsigned long period = (state == WAIT) ? 1000 : (state == FAULT ? 150 : 400);
  if (now - ledTimer >= period) {
    ledTimer = now;
    ledOn = !ledOn;
    digitalWrite(PIN_LED, ledOn ? LED_ON : LED_OFF);
  }
}

// =============================================================================
// setup()
// =============================================================================
void setup() {
  Serial.begin(USB_BAUD);
  HC12.begin(HC12_BAUD, SERIAL_8N1, HC12_RX_PIN, HC12_TX_PIN);

  pinMode(PIN_PUMP_IN1, OUTPUT);
  pinMode(PIN_PUMP_IN2, OUTPUT);
  pinMode(PIN_PUMP_PWM, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LED_OFF);
  pumpStop();

  // Sensorless start assumption (also the fallback baseline): float is hand-launched
  // at the surface with bags empty, so depth = 0 and ballast = 0.
  ballast_ml  = 0.0f;
  est_depth   = 0.0f;
  est_vel     = 0.0f;
  lastModelMs = millis();

  // I2C + Keller sensor bring-up. Never blocks — missing sensor => fallback mode.
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(250);
  scanI2C();

  int attempts = 0;
  do {
    sensor.init();
    if (sensor.isInitialized()) { g_sensorPresent = true; break; }
    Serial.println("Keller LD not detected — check I2C wiring and Vin > 3.65 V");
    delay(500);
  } while (++attempts < 5);

  if (g_sensorPresent) {
    sensor.setFluidDensity(FLUID_DENSITY);
    tareSurface();
    if (CONTROL_USES_SENSOR)
      Serial.println("FLOAT powered on (SENSOR mode) — closed-loop PID depth hold.");
    else
      Serial.println("FLOAT powered on (METHOD mode) — Keller LD present but IGNORED for "
                     "control; flying open-loop ballast schedule + dead reckoning.");
  } else {
    Serial.println("FLOAT powered on (SENSORLESS fallback) — Keller LD absent, running");
    Serial.println("open-loop ballast schedule + dead reckoning. Mission still proceeds.");
  }

  Serial.println("HC-12 listening for any signal to start...");
  Serial.print("Full-stroke time (ms): "); Serial.println(FULL_STROKE_MS);
  Serial.print("Neutral ballast (mL):  "); Serial.println(neutralBallastMl());
}

// =============================================================================
// loop()  —  mission state machine (A1), driven by depthNow()
// =============================================================================
void loop() {
  heartbeat();
  updateModel();                    // always keep the fallback estimate valid
  if (g_sensorPresent) readSensor();
  checkSensorHealth();

  // USB telemetry (1 Hz): active depth source + tracked ballast.
  static unsigned long dbgTimer = 0;
  if (isActiveMission(state) && millis() - dbgTimer >= 1000) {
    dbgTimer = millis();
    Serial.print(useSensor() ? "[SENSOR] depth " : "[EST] depth ");
    Serial.print(depthNow(), 2);
    Serial.print(" m  est_vel "); Serial.print(est_vel, 3);
    Serial.print(" m/s  ballast "); Serial.print(ballast_ml, 1);
    Serial.println(" mL");
  }

  switch (state) {

    // A1: wait for any HC-12 signal (or USB byte on the bench), then dive.
    case WAIT: {
      if (millis() - continuousTimer >= 1000) {
        continuousTimer = millis();
        Serial.print(useSensor() ? "WAITING (sensor) | depth=" : "WAITING (fallback) | depth=");
        Serial.print(depthNow(), 2);
        Serial.print(" m  ballast=");
        Serial.print(ballast_ml, 1);
        Serial.print(" mL  t=");
        Serial.print(millis() / 1000);
        Serial.println("s");
      }

      Stream* in = nullptr;
      if (HC12.available()) in = &HC12;
#ifdef BENCH_TEST
      else if (Serial.available()) in = &Serial;
#endif
      if (in) {
        while (in->available()) in->read();
        missionStart = millis();
        est_depth = 0.0f;                 // launch datum: surface = 0 at mission start
        est_vel   = 0.0f;
        Serial.println("Signal received — diving.");
        enterPhase(DIVE);
      }
      break;
    }

    // A1: descend. SENSOR -> PID to 2.5 m. FALLBACK -> pump IN for a calibrated
    //     run time (fills the bag -> heavy -> sinks).
    case DIVE: {
      serviceContinuousLog();
      bool reached;
      if (useSensor()) {
        controlDepth(DEEP_DEPTH);
        reached = (depthNow() >= DEEP_DEPTH);
      } else {
        servoBallast(DIVE_BALLAST_ML);                 // pump IN toward a full bag
        reached = (millis() - phaseStart >= DIVE_RUN_MS);
      }
      if (reached) {
        Serial.println(useSensor() ? "Reached 2.5 m — holding."
                                   : "Dive run complete — holding deep.");
        enterPhase(HOLD_DEEP);
      } else if (millis() - phaseStart > DIVE_TIMEOUT_MS) {
        enterFault("dive timeout");
      }
      break;
    }

    // A1: hold ~2.5 m for 30 s. SENSOR -> in-band band hold. FALLBACK -> timed hold.
    case HOLD_DEEP: {
      bool done = useSensor() ? serviceHold(DEEP_DEPTH, false) : serviceTimedHold();
      if (done) {
        Serial.println("Deep hold complete — rising.");
        enterPhase(RISE);
      } else if (millis() - phaseStart > HOLD_TIMEOUT_MS) {
        enterFault("deep hold timeout — cannot hold band");
      }
      break;
    }

    // A1: ascend. SENSOR -> PID to 0.40 m. FALLBACK -> pump OUT for a calibrated
    //     run time (empties the bag -> lighter -> rises).
    case RISE: {
      serviceContinuousLog();
      bool reached;
      if (useSensor()) {
        if (depthNow() <= ASCENT_FLOOR) pumpIn(PWM_MAX);    // surface guard
        else                            controlDepth(SHALLOW_DEPTH);
        reached = (depthNow() <= SHALLOW_DEPTH);
      } else {
        servoBallast(RISE_BALLAST_ML);                 // pump OUT toward an empty bag
        reached = (millis() - phaseStart >= RISE_RUN_MS);
      }
      if (reached) {
        Serial.println(useSensor() ? "Reached 0.40 m — holding."
                                   : "Rise run complete — holding shallow.");
        enterPhase(HOLD_SHALLOW);
      } else if (millis() - phaseStart > RISE_TIMEOUT_MS) {
        enterFault("rise timeout");
      }
      break;
    }

    // A1: hold ~0.40 m for 30 s, then next profile or finish.
    case HOLD_SHALLOW: {
      bool done = useSensor() ? serviceHold(SHALLOW_DEPTH, true) : serviceTimedHold();
      if (done) {
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
      } else if (millis() - phaseStart > HOLD_TIMEOUT_MS) {
        enterFault("shallow hold timeout — cannot hold band");
      }
      break;
    }

    // A5: transmit the buffered packets discretely, one at a time.
    case DONE: {
      pumpStop();
      transmitNext();
      break;
    }

    // A6: expel all ballast to surface, then stop and still transmit what we logged.
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
