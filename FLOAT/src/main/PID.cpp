// PID.cpp — implementation of the depth-hold controller (see PID.h).
#include "PID.h"

PID::PID(double kp, double ki, double kd, double outMin, double outMax)
    : _kp(kp), _ki(ki), _kd(kd),
      _outMin(outMin), _outMax(outMax),
      _integral(0.0), _prevError(0.0), _firstStep(true) {}

void PID::reset() {
  _integral  = 0.0;
  _prevError = 0.0;
  _firstStep = true;
}

void PID::setGains(double kp, double ki, double kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

double PID::compute(double error, double dt) {
  if (dt <= 0.0) dt = 1e-3;  // guard against a zero/negative timestep

  // Derivative on error. Skip on the first step so a cold start (large
  // (error - 0)/dt) does not produce a derivative kick.
  double derivative = _firstStep ? 0.0 : (error - _prevError) / dt;
  _prevError = error;
  _firstStep = false;

  // Tentative integral including this step.
  double candidateIntegral = _integral + error * dt;
  double output = _kp * error + _ki * candidateIntegral + _kd * derivative;

  // Output limiting + conditional-integration anti-windup (A2):
  // only commit the new integral when the output is NOT saturated, OR when the
  // current error would push the output back out of saturation. This stops the
  // integral from accumulating while the pump is already maxed out.
  if (output > _outMax) {
    output = _outMax;
    if (error < 0.0) _integral = candidateIntegral;  // error pulls back -> allow
  } else if (output < _outMin) {
    output = _outMin;
    if (error > 0.0) _integral = candidateIntegral;
  } else {
    _integral = candidateIntegral;
  }

  return output;
}
