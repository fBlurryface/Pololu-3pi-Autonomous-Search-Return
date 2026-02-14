#pragma once
#include <Arduino.h>
#include <math.h>

#include "Motors3pi.h"
#include "Encoders3pi.h"
#include "PID3pi.h"

namespace WheelSpeed3pi {

// ---------------- Configuration (defaults = your numbers) ----------------
struct Config {
  // Feedforward
  float dzL = 19.0f;
  float dzR = 15.0f;
  float aL  = 16.0f;   // cps per PWM
  float aR  = 16.5f;

  // PID gains
  float kpL = 0.10f, kiL = 0.60f, kdL = 0.00f;
  float kpR = 0.10f, kiR = 0.70f, kdR = 0.00f;

  // PID limits
  float iMin = -200.0f;
  float iMax =  200.0f;

  float dTauL = 0.00f;
  float dTauR = 0.00f;

  // Timing
  uint16_t ctrlDtMs   = 20;    // control period (also speedUpdate period)
  uint16_t speedWinMs = 100;   // encoder speed window

  // Stop threshold
  float stopEpsCps = 0.5f;

  // Motor forward levels (optional)
  bool leftForwardLevel  = LOW;
  bool rightForwardLevel = LOW;

  // Debug options (optional)
  bool  pidLimitEnableL = false;
  bool  pidLimitEnableR = false;
  float pidLimitAbsL    = 80.0f;   // symmetric clamp on PID output
  float pidLimitAbsR    = 80.0f;

  bool  pidDirGateEnableL = false; // target>0 forbid negative pid; target<0 forbid positive pid
  bool  pidDirGateEnableR = false;
};

// ---------------- Internal controller per wheel ----------------
class _Wheel {
public:
  void init(const Config& cfg, bool isLeft, float dt_s) {
    const float kp = isLeft ? cfg.kpL : cfg.kpR;
    const float ki = isLeft ? cfg.kiL : cfg.kiR;
    const float kd = isLeft ? cfg.kdL : cfg.kdR;

    dz_ = isLeft ? cfg.dzL : cfg.dzR;
    A_  = isLeft ? cfg.aL  : cfg.aR;
    if (A_ < 1e-6f) A_ = 1e-6f;

    stopEps_ = cfg.stopEpsCps;

    pid_.setGains(kp, ki, kd);
    pid_.setDt(dt_s);
    pid_.setOutputLimits(-255.0f, 255.0f); // correction in PWM units
    pid_.setIntegralLimits(cfg.iMin, cfg.iMax);
    pid_.setDerivativeOnMeasurement(true);
    pid_.setDerivativeFilterTau(isLeft ? cfg.dTauL : cfg.dTauR);
    pid_.reset(0.0f);

    // debug
    pidLimitEnable_   = isLeft ? cfg.pidLimitEnableL : cfg.pidLimitEnableR;
    pidLimitAbs_      = isLeft ? cfg.pidLimitAbsL    : cfg.pidLimitAbsR;
    pidDirGateEnable_ = isLeft ? cfg.pidDirGateEnableL : cfg.pidDirGateEnableR;

    running_ = false;
    targetCps_ = 0.0f;
  }

  void setTarget(float cps) { targetCps_ = cps; }

  void setPidLimit(bool en, float absPwm) { pidLimitEnable_ = en; pidLimitAbs_ = absPwm; }
  void setPidDirGate(bool en) { pidDirGateEnable_ = en; }

  int16_t step(float measCps) {
    // stop handling
    if (fabs(targetCps_) <= stopEps_) {
      if (running_) {
        pid_.reset(measCps);
        running_ = false;
      } else {
        pid_.resetIntegral();
      }
      return 0;
    }
    if (!running_) {
      pid_.reset(measCps);
      running_ = true;
    }

    // feedforward
    const float dir = (targetCps_ >= 0.0f) ? 1.0f : -1.0f;
    float pwm_ff_mag = dz_ + (fabs(targetCps_) / A_);
    if (pwm_ff_mag > 255.0f) pwm_ff_mag = 255.0f;
    const float pwm_ff = dir * pwm_ff_mag;

    // PID correction
    float pwm_pid = pid_.update(targetCps_, measCps);

    // (1) PID symmetric clamp (optional)
    if (pidLimitEnable_) {
      const float lim = fabs(pidLimitAbs_);
      if (pwm_pid >  lim) pwm_pid =  lim;
      if (pwm_pid < -lim) pwm_pid = -lim;
    }

    // (2) PID direction gate (optional)
    if (pidDirGateEnable_) {
      if (targetCps_ > 0.0f && pwm_pid < 0.0f) pwm_pid = 0.0f;
      if (targetCps_ < 0.0f && pwm_pid > 0.0f) pwm_pid = 0.0f;
    }

    // final PWM
    float out = pwm_ff + pwm_pid;
    if (out > 255.0f) out = 255.0f;
    if (out < -255.0f) out = -255.0f;
    return (int16_t)out;
  }

private:
  PID3pi pid_;
  float dz_ = 0.0f;
  float A_  = 1.0f;
  float stopEps_ = 0.5f;

  float targetCps_ = 0.0f;
  bool  running_   = false;

  // debug
  bool  pidLimitEnable_ = false;
  float pidLimitAbs_    = 80.0f;
  bool  pidDirGateEnable_ = false;
};

// ---------------- Public module (singleton-style API) ----------------
class Controller {
public:
  void begin(const Config& cfg = Config()) {
    cfg_ = cfg;

    motors_.setForwardLevels(cfg_.leftForwardLevel, cfg_.rightForwardLevel);
    motors_.begin();

    Encoders3pi::begin();
    Encoders3pi::speedBegin(cfg_.speedWinMs);

    const float dt_s = (cfg_.ctrlDtMs > 0 ? cfg_.ctrlDtMs : 1) / 1000.0f;
    left_.init(cfg_, true,  dt_s);
    right_.init(cfg_, false, dt_s);

    nextTickUs_ = 0;
    targetL_ = 0.0f;
    targetR_ = 0.0f;
    started_ = true;
  }

  void setTargetCps(float leftCps, float rightCps) {
    targetL_ = leftCps;
    targetR_ = rightCps;
  }

  // Optional debug controls at runtime
  void setPidLimit(bool enable, float absPwm, bool applyLeft = true, bool applyRight = true) {
    if (applyLeft)  left_.setPidLimit(enable, absPwm);
    if (applyRight) right_.setPidLimit(enable, absPwm);
  }
  void setPidDirGate(bool enable, bool applyLeft = true, bool applyRight = true) {
    if (applyLeft)  left_.setPidDirGate(enable);
    if (applyRight) right_.setPidDirGate(enable);
  }

  void coast() { motors_.coast(); }

  // Call this frequently in loop(); it will internally run at fixed ctrlDtMs.
  void update() {
    if (!started_) return;

    const uint32_t periodUs = (uint32_t)(cfg_.ctrlDtMs > 0 ? cfg_.ctrlDtMs : 1) * 1000UL;
    const uint32_t nowUs = micros();

    if (nextTickUs_ == 0) {
      nextTickUs_ = nowUs + periodUs;
      return;
    }
    if ((int32_t)(nowUs - nextTickUs_) < 0) return;

    // advance schedule; if lag too much, resync to avoid burst updates
    nextTickUs_ += periodUs;
    if ((int32_t)(nowUs - nextTickUs_) > (int32_t)(periodUs * 5UL)) {
      nextTickUs_ = nowUs + periodUs;
    }

    // 20ms tick: update speed estimator then control
    Encoders3pi::speedUpdate();

    const float vL = Encoders3pi::getSpeedLeftCPS();
    const float vR = Encoders3pi::getSpeedRightCPS();

    left_.setTarget(targetL_);
    right_.setTarget(targetR_);

    const int16_t pwmL = left_.step(vL);
    const int16_t pwmR = right_.step(vR);

    motors_.set(pwmL, pwmR);
  }

private:
  Config cfg_;
  Motors3pi motors_{LOW, LOW};
  _Wheel left_, right_;

  float targetL_ = 0.0f;
  float targetR_ = 0.0f;

  uint32_t nextTickUs_ = 0;
  bool started_ = false;
};

// Singleton accessor (avoids global definition issues)
inline Controller& _instance() {
  static Controller inst;
  return inst;
}

// ---------------- User-facing API ----------------
inline void begin(const Config& cfg = Config()) { _instance().begin(cfg); }
inline void setTargetCps(float leftCps, float rightCps) { _instance().setTargetCps(leftCps, rightCps); }
inline void update() { _instance().update(); }
inline void coast() { _instance().coast(); }

// Optional debug API (runtime)
inline void setPidLimit(bool enable, float absPwm, bool applyLeft = true, bool applyRight = true) {
  _instance().setPidLimit(enable, absPwm, applyLeft, applyRight);
}
inline void setPidDirGate(bool enable, bool applyLeft = true, bool applyRight = true) {
  _instance().setPidDirGate(enable, applyLeft, applyRight);
}

} // namespace WheelSpeed3pi
