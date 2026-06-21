// PID.h — depth-hold controller for the FLOAT buoyancy engine.
//
// Requirements addressed:
//   A2 — "PID holds depth tightly inside ±33 cm with integral anti-windup and
//         output limits". This class clamps its output and uses conditional
//         integration so the integral term cannot wind up while saturated.
//   A7 — replaces `#include "PID.cpp"`; exposes ONLY the controller (no extra
//         setup()/loop(), no duplicate MS5837 `sensor`).
//
// Convention used by the firmware:
//   error  = setpoint_depth - measured_depth   (metres)
//   output : signed effort in [outMin, outMax]; the caller maps
//            output > 0 -> pump water IN  (heavier -> descend)
//            output < 0 -> pump water OUT (lighter -> ascend)
#pragma once

class PID {
public:
  PID(double kp, double ki, double kd, double outMin, double outMax);

  // Run one control step. `dt` is the elapsed time in seconds since the last
  // call. Returns the clamped controller output.
  double compute(double error, double dt);

  // Clear integral/derivative memory — call on every state transition so a new
  // hold does not inherit the previous phase's accumulated error.
  void reset();

  // Re-tune live (A2: gains are placeholders until tuned in water).
  void setGains(double kp, double ki, double kd);

  double kp() const { return _kp; }
  double ki() const { return _ki; }
  double kd() const { return _kd; }

private:
  double _kp, _ki, _kd;
  double _outMin, _outMax;
  double _integral;
  double _prevError;
  bool   _firstStep;
};
