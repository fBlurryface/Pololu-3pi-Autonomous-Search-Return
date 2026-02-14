// ============================================================
// Arduino auto-prototype FIX (must be placed before all #includes)
// ============================================================
// Arduino may auto-generate function prototypes before type definitions.
// Forward-declare the enum here (like your main code does) to avoid ordering issues.
enum class DemoState : unsigned char;

#include <Arduino.h>
#include <math.h>

#define ENCODERS3PI_IMPLEMENTATION
#include "Encoders3pi.h"

#include "WheelSpeedCtrl3pi.h"

// ============================================================
// Motion params (use your current values)
// ============================================================

struct MotionParams {
  float cruiseAvgUp    = 220.0f;
  float cruiseAvgDown  = 240.0f;

  float edgeDiffUp     = 1200.0f;
  float edgeDiffDown   = 1200.0f;

  float targetClampCps = 250.0f;

  float cruisePidClampAbs = 60.0f;
  float edgePidClampAbs   = 35.0f;

  uint16_t ctrlDtMs   = 20;
  uint16_t speedWinMs = 100;

  bool leftForwardLevel  = LOW;
  bool rightForwardLevel = LOW;
};

MotionParams MP;

// ============================================================
// Accel limiter (same as your main)
// ============================================================

struct AccelLimiter {
  float y = 0.0f;
  float accelUp = 220.0f;
  float accelDown = 350.0f;
  uint32_t lastUs = 0;

  void reset(float value = 0.0f) { y = value; lastUs = micros(); }

  float update(float x) {
    uint32_t now = micros();
    float dt = (lastUs == 0) ? 0.0f : (now - lastUs) * 1e-6f;
    lastUs = now;
    if (dt <= 0.0f) { y = x; return y; }

    float dy = x - y;
    float maxStep = (dy >= 0.0f ? accelUp : accelDown) * dt;
    if (fabsf(dy) > fabsf(maxStep)) dy = copysignf(maxStep, dy);
    y += dy;
    return y;
  }
};

static AccelLimiter limAvg;
static AccelLimiter limDiff;

static inline float clampCps(float x) {
  const float lim = MP.targetClampCps;
  if (x > lim) return lim;
  if (x < -lim) return -lim;
  return x;
}

static inline void setPidClampCruise() {
  WheelSpeed3pi::setPidLimit(true, MP.cruisePidClampAbs, true, true);
}

static inline void setPidClampEdge() {
  WheelSpeed3pi::setPidLimit(true, MP.edgePidClampAbs, true, true);
}

static inline void initShapers() {
  limAvg.accelUp   = MP.cruiseAvgUp;
  limAvg.accelDown = MP.cruiseAvgDown;

  limDiff.accelUp   = MP.edgeDiffUp;
  limDiff.accelDown = MP.edgeDiffDown;

  limAvg.reset(0.0f);
  limDiff.reset(0.0f);
}

// avg/diff shaping -> wheel targets -> WheelSpeed update
static inline void shapedMotionUpdate(float avgDes, float diffDes,
                                      float* outAvgCmd, float* outDiffCmd,
                                      float* outTgtL, float* outTgtR) {
  const float avgCmd  = limAvg.update(avgDes);
  const float diffCmd = limDiff.update(diffDes);

  const float tgtL = clampCps(avgCmd - diffCmd);
  const float tgtR = clampCps(avgCmd + diffCmd);

  WheelSpeed3pi::setTargetCps(tgtL, tgtR);
  WheelSpeed3pi::update();

  if (outAvgCmd)  *outAvgCmd  = avgCmd;
  if (outDiffCmd) *outDiffCmd = diffCmd;
  if (outTgtL)    *outTgtL    = tgtL;
  if (outTgtR)    *outTgtR    = tgtR;
}

// ============================================================
// Demo FSM: show shaping under many command patterns
// ============================================================

enum class DemoState : unsigned char {
  IDLE = 0,

  FWD_STEP,
  STOP1,
  REV_STEP,
  STOP2,

  PIVOT_R,
  STOP3,
  PIVOT_L,
  STOP4,

  ARC_L,
  ARC_R,

  CLAMP_AVG,
  CLAMP_DIFF,

  STOP_END
};

static DemoState gState = DemoState::IDLE;
static uint32_t  stateStartMs = 0;
static uint32_t  missionStartMs = 0;

static uint32_t lastCtrlMs  = 0;
static uint32_t lastPrintMs = 0;

static inline uint32_t stateDurationMs(DemoState s) {
  switch (s) {
    case DemoState::IDLE:       return 1000;

    case DemoState::FWD_STEP:   return 3000;
    case DemoState::STOP1:      return 2000;
    case DemoState::REV_STEP:   return 3000;
    case DemoState::STOP2:      return 1500;

    case DemoState::PIVOT_R:    return 2000;
    case DemoState::STOP3:      return 1000;
    case DemoState::PIVOT_L:    return 2000;
    case DemoState::STOP4:      return 1000;

    case DemoState::ARC_L:      return 3000;
    case DemoState::ARC_R:      return 3000;

    case DemoState::CLAMP_AVG:  return 2000;
    case DemoState::CLAMP_DIFF: return 2000;

    case DemoState::STOP_END:   return 2000;
  }
  return 1000;
}

static inline DemoState nextState(DemoState s) {
  switch (s) {
    case DemoState::IDLE:       return DemoState::FWD_STEP;

    case DemoState::FWD_STEP:   return DemoState::STOP1;
    case DemoState::STOP1:      return DemoState::REV_STEP;
    case DemoState::REV_STEP:   return DemoState::STOP2;
    case DemoState::STOP2:      return DemoState::PIVOT_R;

    case DemoState::PIVOT_R:    return DemoState::STOP3;
    case DemoState::STOP3:      return DemoState::PIVOT_L;
    case DemoState::PIVOT_L:    return DemoState::STOP4;
    case DemoState::STOP4:      return DemoState::ARC_L;

    case DemoState::ARC_L:      return DemoState::ARC_R;
    case DemoState::ARC_R:      return DemoState::CLAMP_AVG;

    case DemoState::CLAMP_AVG:  return DemoState::CLAMP_DIFF;
    case DemoState::CLAMP_DIFF: return DemoState::STOP_END;

    case DemoState::STOP_END:   return DemoState::IDLE;
  }
  return DemoState::IDLE;
}

static inline void stateCommand(DemoState s, float* avgDes, float* diffDes, bool* edgeClamp) {
  *avgDes = 0.0f;
  *diffDes = 0.0f;
  *edgeClamp = false;

  switch (s) {
    case DemoState::IDLE:
      *avgDes = 0.0f; *diffDes = 0.0f; *edgeClamp = false;
      break;

    case DemoState::FWD_STEP:
      *avgDes = 300.0f; *diffDes = 0.0f; *edgeClamp = false;
      break;

    case DemoState::STOP1:
      *avgDes = 0.0f; *diffDes = 0.0f; *edgeClamp = false;
      break;

    case DemoState::REV_STEP:
      *avgDes = -300.0f; *diffDes = 0.0f; *edgeClamp = false;
      break;

    case DemoState::STOP2:
      *avgDes = 0.0f; *diffDes = 0.0f; *edgeClamp = false;
      break;

    case DemoState::PIVOT_R:
      *avgDes = 0.0f; *diffDes = -200.0f; *edgeClamp = true;
      break;

    case DemoState::STOP3:
      *avgDes = 0.0f; *diffDes = 0.0f; *edgeClamp = true;
      break;

    case DemoState::PIVOT_L:
      *avgDes = 0.0f; *diffDes = 200.0f; *edgeClamp = true;
      break;

    case DemoState::STOP4:
      *avgDes = 0.0f; *diffDes = 0.0f; *edgeClamp = true;
      break;

    case DemoState::ARC_L:
      *avgDes = 220.0f; *diffDes = 80.0f; *edgeClamp = false;
      break;

    case DemoState::ARC_R:
      *avgDes = 220.0f; *diffDes = -80.0f; *edgeClamp = false;
      break;

    case DemoState::CLAMP_AVG:
      *avgDes = 500.0f; *diffDes = 0.0f; *edgeClamp = false;
      break;

    case DemoState::CLAMP_DIFF:
      *avgDes = 0.0f; *diffDes = 400.0f; *edgeClamp = true;
      break;

    case DemoState::STOP_END:
      *avgDes = 0.0f; *diffDes = 0.0f; *edgeClamp = false;
      break;
  }
}

static inline void enterState(DemoState s, uint32_t nowMs) {
  gState = s;
  stateStartMs = nowMs;

  float avgDes = 0.0f, diffDes = 0.0f;
  bool edgeClamp = false;
  stateCommand(gState, &avgDes, &diffDes, &edgeClamp);

  if (edgeClamp) setPidClampEdge();
  else setPidClampCruise();
}

// ============================================================
// Setup / Loop
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  WheelSpeed3pi::Config cfg;
  cfg.ctrlDtMs = MP.ctrlDtMs;
  cfg.speedWinMs = MP.speedWinMs;
  cfg.leftForwardLevel  = MP.leftForwardLevel;
  cfg.rightForwardLevel = MP.rightForwardLevel;

  cfg.pidLimitEnableL = true; cfg.pidLimitAbsL = MP.cruisePidClampAbs;
  cfg.pidLimitEnableR = true; cfg.pidLimitAbsR = MP.cruisePidClampAbs;

  WheelSpeed3pi::begin(cfg);

  // Ensure encoder speed window matches this demo (harmless even if WheelSpeed already set it)
  Encoders3pi::speedBegin(MP.speedWinMs);

  initShapers();
  setPidClampCruise();

  missionStartMs = millis();
  enterState(DemoState::IDLE, missionStartMs);

  // CSV: t_ms,stateId,avgDes,diffDes,avgCmd,diffCmd,tgtL,tgtR,vL,vR
  Serial.println("t_ms,state,avgDes,diffDes,avgCmd,diffCmd,tgtL,tgtR,vL,vR");
}

void loop() {
  const uint32_t nowMs = millis();

  // FSM transition by time
  if ((uint32_t)(nowMs - stateStartMs) >= stateDurationMs(gState)) {
    enterState(nextState(gState), nowMs);
  }

  // Control update at fixed cadence
  if ((uint32_t)(nowMs - lastCtrlMs) >= MP.ctrlDtMs) {
    lastCtrlMs = nowMs;

    float avgDes = 0.0f, diffDes = 0.0f;
    bool edgeClamp = false;
    stateCommand(gState, &avgDes, &diffDes, &edgeClamp);

    float avgCmd = 0.0f, diffCmd = 0.0f, tgtL = 0.0f, tgtR = 0.0f;
    shapedMotionUpdate(avgDes, diffDes, &avgCmd, &diffCmd, &tgtL, &tgtR);

    // Print at 10 Hz
    if ((uint32_t)(nowMs - lastPrintMs) >= 100) {
      lastPrintMs = nowMs;

      const float vL = Encoders3pi::getSpeedLeftCPS();
      const float vR = Encoders3pi::getSpeedRightCPS();

      Serial.print((uint32_t)(nowMs - missionStartMs));
      Serial.print(',');
      Serial.print((unsigned int)gState);
      Serial.print(',');
      Serial.print(avgDes, 3);
      Serial.print(',');
      Serial.print(diffDes, 3);
      Serial.print(',');
      Serial.print(avgCmd, 3);
      Serial.print(',');
      Serial.print(diffCmd, 3);
      Serial.print(',');
      Serial.print(tgtL, 3);
      Serial.print(',');
      Serial.print(tgtR, 3);
      Serial.print(',');
      Serial.print(vL, 3);
      Serial.print(',');
      Serial.println(vR, 3);
    }
  }
}
