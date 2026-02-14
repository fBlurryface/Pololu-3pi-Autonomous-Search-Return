#ifndef PID3PI_H
#define PID3PI_H

#include <Arduino.h>

/*
  PID3pi.h  (header-only)

  - Full PID: u = Kp*e + Ki*∫e dt + Kd*de/dt
  - Supports:
    1) Output clamping: outMin/outMax
    2) Integral clamping (a minimal but very effective anti-windup)
    3) D-on-measurement (enabled by default; recommended for wheel-speed control)
    4) First-order low-pass filter on the derivative term (tau, seconds; tau=0 means disabled)
  - You can set Kd=0 and use it as a PI controller; the D-term interface is kept.

  Usage suggestions (wheel-speed control):
    - Keep the sampling period dt fixed (e.g. 0.02s = 20ms)
    - D-on-measurement = true
    - Try tau in the range of 0.02~0.08s (or start with 0 to disable)
*/

class PID3pi {
public:
  PID3pi() = default;

  PID3pi(float kp, float ki, float kd, float dt_s) {
    setGains(kp, ki, kd);
    setDt(dt_s);
  }

  // ----- Parameter setters -----
  inline void setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  inline void setDt(float dt_s) {
    // Prevent division by zero
    dt_ = (dt_s > 1e-6f) ? dt_s : 1e-6f;
    // Update filter coefficient
    updateDerivFilterCoeff_();
  }

  inline float getDt() const {
    return dt_; 
  }

  inline void setOutputLimits(float outMin, float outMax)  {
    outMin_ = outMin;
    outMax_ = outMax;
    if (outMin_ > outMax_) { float t = outMin_; outMin_ = outMax_; outMax_ = t; }
  }

  inline void setIntegralLimits(float iMin, float iMax) {
    iMin_ = iMin;
    iMax_ = iMax;
    if (iMin_ > iMax_) { float t = iMin_; iMin_ = iMax_; iMax_ = t; }
  }

  // tau: derivative filter time constant (seconds); tau<=0 disables filtering
  inline void setDerivativeFilterTau(float tau_s) {
    tau_ = (tau_s > 0.0f) ? tau_s : 0.0f;
    updateDerivFilterCoeff_();
  }

  // true: take derivative on "measurement" (recommended; avoids D kick on setpoint steps)
  // false: take derivative on "error" (textbook form, but setpoint steps cause D kick)
  inline void setDerivativeOnMeasurement(bool enable) { 
    dOnMeas_ = enable; 
  }

  // ----- State control -----
  inline void reset(float measurement = 0.0f) {
    integ_ = 0.0f;
    prevErr_ = 0.0f;
    prevMeas_ = measurement;
    dFilt_ = 0.0f;
    initialized_ = false;
  }

  // Clear only the integrator (sometimes very handy for debugging)
  inline void resetIntegral() { 
    integ_ = 0.0f;
  }

  // ----- Core: run one PID update -----
  inline float update(float setpoint, float measurement) {
    // Initialization: avoid derivative spike on the first step
    if (!initialized_) {
      prevMeas_ = measurement;
      prevErr_ = setpoint - measurement;
      dFilt_ = 0.0f;
      initialized_ = true;
    }

    const float err = setpoint - measurement;

    // I: integral (with clamping)
    integ_ += err * dt_;
    if (integ_ > iMax_) integ_ = iMax_;
    if (integ_ < iMin_) integ_ = iMin_;

    // D: derivative (optionally on measurement or on error)
    float derivRaw;
    if (dOnMeas_) {
      // The measurement part of d/dt(setpoint - meas): -d(meas)/dt
      derivRaw = -(measurement - prevMeas_) / dt_;

    }else {

      derivRaw = (err - prevErr_) / dt_;

    }

    // Derivative filtering (first-order low-pass): dFilt = a*dFilt + (1-a)*derivRaw
    if (tau_ > 0.0f) {

      dFilt_ = a_ * dFilt_ + (1.0f - a_) * derivRaw;

    }else {

      dFilt_ = derivRaw;

    }

    // PID output
    float u = kp_ * err + ki_ * integ_ + kd_ * dFilt_;

    // Output clamping
    if (u > outMax_) u = outMax_;
    if (u < outMin_) u = outMin_;

    // Update history
    prevErr_ = err;
    prevMeas_ = measurement;

    return u;
  }

private:
  inline void updateDerivFilterCoeff_() {
    // a = tau/(tau+dt)
    if (tau_ > 0.0f)
      a_ = tau_ / (tau_ + dt_);
    else
      a_ = 0.0f;
  }

  // gains
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;

  // timing
  float dt_ = 0.02f;       // seconds

  // limits
  float outMin_ = -255.0f;
  float outMax_ =  255.0f;
  float iMin_   = -200.0f;
  float iMax_   =  200.0f;

  // state
  float integ_ = 0.0f;
  float prevErr_ = 0.0f;
  float prevMeas_ = 0.0f;

  // derivative filter
  bool  dOnMeas_ = true;
  float tau_ = 0.0f;        // seconds; 0 = off
  float a_ = 0.0f;          // filter coefficient
  float dFilt_ = 0.0f;

  bool initialized_ = false;
};

#endif // PID3PI_H

