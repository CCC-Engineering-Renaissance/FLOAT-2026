// =============================================================================
// FLOAT-2026 firmware  —  MATE "MATE Floats! Under the Ice" vertical profiler
// SENSORLESS build: NO pressure sensor. Single setup()/loop(), Arduino framework,
// Seeed Studio XIAO ESP32-S3.
//
// HOW THIS WORKS WITHOUT A DEPTH SENSOR
// -------------------------------------
// There is no way to *measure* depth without a pressure sensor, so this firmware
// does NOT close a loop on depth. Instead it runs OPEN-LOOP buoyancy scheduling:
//
//   1. The pump is positive-displacement (peristaltic), so water moved = flow_rate
//      * run_time. We integrate pump run-time to track how much water is in the
//      IV-bag reservoir (`ballast_ml`) — this IS observable and reliable.
//   2. Each mission phase servos `ballast_ml` to a target volume (heavy = dive,
//      light = rise, neutral = hold). A real closed loop, but on BALLAST not depth.
//   3. A dead-reckoning model integrates the equation of motion (buoyancy - weight
//      - drag) to ESTIMATE depth/velocity, used to sequence phases and fill the
//      logged graph. It is an ESTIMATE, not a measurement.
//
// HONEST LIMITATIONS — READ THIS
//   * The depth estimate DRIFTS. A sub-1% trim/volume error walks the float out of
//     the ±33 cm band in a few seconds with no way to correct it. "Hold" is
//     best-effort, not a guaranteed ±33 cm.
//   * Calibrate phase timing in the pool: measure the real descend/ascend time and
//     set DIVE_TIMEOUT_MS / RISE_TIMEOUT_MS so phases end even if the model drifts.
//   * BUOYANCY BUDGET still applies (see constants): at 12 lb in a 2.56 L hull the
//     float is ~2.8 kg negative and physically cannot rise. No code fixes that.
// =============================================================================
#include <Arduino.h>

// -----------------------------------------------------------------------------
// Pin map for Seeed Studio XIAO ESP32-S3.
//   HC-12 radio : Serial1 -> D6/GPIO43 (XIAO TX -> HC-12 RX),
//                           D7/GPIO44 (XIAO RX <- HC-12 TX)
//   L298N ch A  : IN1=D1/GPIO2, IN2=D2/GPIO3, ENA=D0/GPIO1 (PWM speed/enable).
//                 VM = 12 V battery, logic VCC = XIAO 3V3, GND common.
//   LED         : XIAO user LED = GPIO21 (active-LOW).
// (The former I2C pins for the Keller sensor are no longer used — sensorless.)
// -----------------------------------------------------------------------------
#define PIN_PUMP_IN1  2   // D1/GPIO2: L298N IN1 (direction)
#define PIN_PUMP_IN2  3   // D2/GPIO3: L298N IN2 (direction)
#define PIN_PUMP_PWM  1   // D0/GPIO1: L298N ENA (PWM speed/enable)
#define PIN_LED       21  // XIAO S3 user LED (active-LOW)
#define HC12_RX_PIN   44  // D7/GPIO44: XIAO RX <- HC-12 TX
#define HC12_TX_PIN   43  // D6/GPIO43: XIAO TX -> HC-12 RX

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

static const int   PWM_MAX        = 255;     // pump speed ceiling

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
//   still sink. DIVE will work; RISE/HOLD will time out into FAULT. The model below
//   reflects this honestly (its estimated depth just keeps increasing). The fix is
//   physical: ~5.6 lb mass OR ~5.4 L displacement (add a sealed ~2.85 L air chamber).

// Per-phase ballast targets (mL). Dive = heaviest, rise = lightest, hold = neutral.
static const float DIVE_BALLAST_ML = BALLAST_ML;   // full -> sink
static const float RISE_BALLAST_ML = 0.0f;         // empty -> rise
static const float BALLAST_TOL_ML  = 2.0f;         // servo deadband

// Phase timeouts. PRIMARY phase end is the estimated-depth trigger; these are the
// fallbacks so a phase always ends even when the estimate drifts. CALIBRATE in pool.
static const unsigned long DIVE_TIMEOUT_MS = FULL_STROKE_MS + 90000;  // ~166 s
static const unsigned long RISE_TIMEOUT_MS = FULL_STROKE_MS + 90000;  // ~166 s
static const unsigned long HOLD_TIMEOUT_MS = 120000;                  // give up holding -> FAULT

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
  float depth;       // m  (ESTIMATED, dead-reckoned — not measured)
  float kpa;         // kPa (derived from estimated depth)
};

static State  state = WAIT;
static int    cycle = 0;

static Packet packets[MAX_PACKETS];
static int    packetCount    = 0;
static int    transmitIndex  = 0;

// -----------------------------------------------------------------------------
// Sensorless estimator state (dead reckoning)
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
static unsigned long continuousTimer= 0;
static unsigned long holdPacketTimer= 0;
static unsigned long holdStart      = 0;
static unsigned long phaseStart     = 0;
static unsigned long transmitTimer  = 0;
static unsigned long faultStart     = 0;
static unsigned long ledTimer       = 0;

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
// Ballast servo: drive `ballast_ml` toward a target volume by running the pump.
// Because the pump is positive-displacement, run-time IS volume — this loop is
// reliable (unlike the depth estimate). Returns true when within the deadband.
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
// Sensorless model update (dead reckoning). Integrates ballast from pump run-time,
// then integrates the vertical equation of motion to estimate velocity and depth.
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

  // 2) Vertical dynamics.
  float V_ball = ballast_ml * 1e-6f;
  float mass   = FLOAT_MASS_KG + FLUID_DENSITY * V_ball;
  float F_down = mass * GRAVITY - FLUID_DENSITY * GRAVITY * V_HULL_M3;
  float F_drag = 0.5f * FLUID_DENSITY * DRAG_CD * A_CROSS * est_vel * fabsf(est_vel);
  float m_eff  = mass + ADDED_MASS_CA * FLUID_DENSITY * V_HULL_M3;
  float a      = (F_down - F_drag) / m_eff;

  est_vel   += a * dt;
  est_depth += est_vel * dt;
  if (est_depth < 0.0f) {                     // hit the surface
    est_depth = 0.0f;
    if (est_vel < 0.0f) est_vel = 0.0f;
  }

  // Estimated gauge pressure from estimated depth (P = rho*g*h), kPa.
  est_kpa = FLUID_DENSITY * GRAVITY * est_depth / 1000.0f;
}

// -----------------------------------------------------------------------------
// Logging (A3) — bounded buffer. NOTE: depth/pressure here are ESTIMATED.
// -----------------------------------------------------------------------------
static void logPacket() {
  if (packetCount >= MAX_PACKETS) return;
  unsigned long tsec = missionStart ? (millis() - missionStart) / 1000 : 0;
  packets[packetCount++] = { tsec, est_depth, est_kpa };
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
// Hold helper: servo to neutral and count 30 s only while the ESTIMATE is in band;
// reset the clock the instant it leaves. Logs a packet every 5 s while in band.
// `protectSurface` adds ballast if the estimate rises above the ascent floor.
// Returns true on a completed in-band 30 s hold.
// -----------------------------------------------------------------------------
static bool serviceHold(float setpoint, bool protectSurface) {
  unsigned long now = millis();

  if (protectSurface && est_depth <= ASCENT_FLOOR) {
    pumpIn(PWM_MAX);                       // too shallow: add ballast, sink back
  } else {
    servoBallast(neutralBallastMl());      // best-effort neutral trim
  }

  if (fabsf(est_depth - setpoint) <= BAND) {
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
    digitalWrite(PIN_LED, ledOn ? HIGH : LOW);   // XIAO LED is active-LOW (cosmetic)
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
  pumpStop();

  // Sensorless start assumption: the float is hand-launched at the surface with
  // the bags empty, so depth = 0 and ballast = 0. If your reservoir is pre-filled,
  // set ballast_ml here to match, or empty it before launch.
  ballast_ml  = 0.0f;
  est_depth   = 0.0f;
  est_vel     = 0.0f;
  lastModelMs = millis();

  Serial.println("FLOAT powered on (SENSORLESS) — HC-12 listening for any signal to start...");
  Serial.print("Full-stroke time (ms): "); Serial.println(FULL_STROKE_MS);
  Serial.print("Neutral ballast (mL):  "); Serial.println(neutralBallastMl());
}

// =============================================================================
// loop()  —  mission state machine (A1), driven by the dead-reckoning estimate
// =============================================================================
void loop() {
  heartbeat();
  updateModel();                    // refresh ballast + estimated depth/velocity

  // USB telemetry (1 Hz): estimated depth/velocity + tracked ballast.
  static unsigned long dbgTimer = 0;
  if (isActiveMission(state) && millis() - dbgTimer >= 1000) {
    dbgTimer = millis();
    Serial.print("est_depth "); Serial.print(est_depth, 2);
    Serial.print(" m  est_vel "); Serial.print(est_vel, 3);
    Serial.print(" m/s  ballast "); Serial.print(ballast_ml, 1);
    Serial.println(" mL  [ESTIMATED]");
  }

  switch (state) {

    // A1: wait for any HC-12 signal (or USB byte on the bench), then dive.
    case WAIT: {
      if (millis() - continuousTimer >= 1000) {
        continuousTimer = millis();
        Serial.print("WAITING for HC-12 signal | est_depth=");
        Serial.print(est_depth, 2);
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
        Serial.println("Signal received — diving.");
        enterPhase(DIVE);
      }
      break;
    }

    // A1: descend — fill the bags (heaviest) and ride the estimate down to 2.5 m.
    case DIVE: {
      serviceContinuousLog();
      servoBallast(DIVE_BALLAST_ML);
      if (est_depth >= DEEP_DEPTH) {
        Serial.println("Estimated 2.5 m — holding.");
        holdStart = millis();
        holdPacketTimer = millis() - PACKET_INTERVAL;
        enterPhase(HOLD_DEEP);
      } else if (millis() - phaseStart > DIVE_TIMEOUT_MS) {
        enterFault("dive timeout — estimate never reached 2.5 m");
      }
      break;
    }

    // A1: hold ~2.5 m for 30 s (in-band by estimate; reset clock if it drifts out).
    case HOLD_DEEP: {
      if (serviceHold(DEEP_DEPTH, false)) {
        Serial.println("Deep hold complete — rising.");
        enterPhase(RISE);
      } else if (millis() - phaseStart > HOLD_TIMEOUT_MS) {
        enterFault("deep hold timeout — cannot hold band");
      }
      break;
    }

    // A1: ascend — empty the bags (lightest) and ride the estimate up to 0.40 m.
    case RISE: {
      serviceContinuousLog();
      if (est_depth <= ASCENT_FLOOR) pumpIn(PWM_MAX);  // surface guard
      else servoBallast(RISE_BALLAST_ML);
      if (est_depth <= SHALLOW_DEPTH) {
        Serial.println("Estimated 0.40 m — holding.");
        holdStart = millis();
        holdPacketTimer = millis() - PACKET_INTERVAL;
        enterPhase(HOLD_SHALLOW);
      } else if (millis() - phaseStart > RISE_TIMEOUT_MS) {
        enterFault("rise timeout — estimate never reached 0.40 m");
      }
      break;
    }

    // A1: hold ~0.40 m for 30 s with the surface guard, then next profile or finish.
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
      } else if (millis() - phaseStart > HOLD_TIMEOUT_MS) {
        enterFault("shallow hold timeout — cannot hold band");
      }
      break;
    }

    // A5: transmit the buffered (estimated) packets discretely, one at a time.
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
