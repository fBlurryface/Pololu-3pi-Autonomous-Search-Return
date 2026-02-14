#pragma once
#include <Arduino.h>
#include <math.h>
#include "Encoders3pi.h"

// Ultra-basic differential-drive odometry (wrapped integration version):
// - readEncoderDeltas(): read raw encoder deltas (wrap-safe)
// - integrateDeltas(): integrate pose using "corrected delta counts" (e.g., after taking up backlash)
// - update(): compatibility interface (integrate raw directly, no backlash handling)
// - Right turns make theta negative (continuously accumulated)
class OdomDiff2W_Basic {
public:
  struct Config {
    float metersPerCountL = 0.000282149f;
    float metersPerCountR = 0.000282149f;
    float wheelBaseM      = 0.08275f;
  };

  struct Pose {
    float x  = 0.0f;
    float y  = 0.0f;
    float th = 0.0f;   // continuous, right turn negative
  };

  void begin(const Config& cfg, bool resetEncoders = true) {
    cfg_ = cfg;
    if (resetEncoders) Encoders3pi::resetCounts();

    lastRawL_ = Encoders3pi::getCountsLeft();
    lastRawR_ = Encoders3pi::getCountsRight();
    accL_ = accR_ = 0;
    pose_ = Pose{};
    initialized_ = true;
  }

  // Read raw encoder deltas (counts) and update internal lastRaw
  inline void readEncoderDeltas(int16_t* dL_raw, int16_t* dR_raw) {
    if (!initialized_) { if (dL_raw) *dL_raw = 0; if (dR_raw) *dR_raw = 0; return; }

    const int16_t rawL = Encoders3pi::getCountsLeft();
    const int16_t rawR = Encoders3pi::getCountsRight();

    const int16_t dL = (int16_t)(rawL - lastRawL_);
    const int16_t dR = (int16_t)(rawR - lastRawR_);

    lastRawL_ = rawL;
    lastRawR_ = rawR;

    if (dL_raw) *dL_raw = dL;
    if (dR_raw) *dR_raw = dR;
  }

  // Integrate using "corrected delta counts" (you can feed in dL/dR after backlash compensation)
  inline void integrateDeltas(int16_t dL, int16_t dR) {
    if (!initialized_) return;

    accL_ += (int32_t)dL;
    accR_ += (int32_t)dR;

    const float dSL = (float)dL * cfg_.metersPerCountL;
    const float dSR = (float)dR * cfg_.metersPerCountR;

    const float dS  = 0.5f * (dSR + dSL);
    const float dTh = (cfg_.wheelBaseM != 0.0f) ? ((dSR - dSL) / cfg_.wheelBaseM) : 0.0f;

    const float thMid = pose_.th + 0.5f * dTh;
    pose_.x  += dS * cosf(thMid);
    pose_.y  += dS * sinf(thMid);
    pose_.th += dTh;
  }

  // Compatibility interface: integrate raw directly (no backlash handling)
  inline void update() {
    int16_t dL_raw = 0, dR_raw = 0;
    readEncoderDeltas(&dL_raw, &dR_raw);
    integrateDeltas(dL_raw, dR_raw);
  }

  Pose getPose() const { return pose_; }

  void setPose(float x, float y, float th) { pose_.x = x; pose_.y = y; pose_.th = th; }

  void resetAll(float x = 0.0f, float y = 0.0f, float th = 0.0f) {
    Encoders3pi::resetCounts();
    lastRawL_ = Encoders3pi::getCountsLeft();
    lastRawR_ = Encoders3pi::getCountsRight();
    accL_ = accR_ = 0;
    setPose(x, y, th);
    initialized_ = true;
  }

  int32_t getAccCountsL() const { return accL_; }
  int32_t getAccCountsR() const { return accR_; }

private:
  Config cfg_;
  Pose   pose_;

  int16_t lastRawL_ = 0;
  int16_t lastRawR_ = 0;
  int32_t accL_ = 0;
  int32_t accR_ = 0;

  bool initialized_ = false;
};
