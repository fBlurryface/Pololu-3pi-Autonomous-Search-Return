// ===== RectWalk.ino =====
// Walk a rectangle: 30cm x 20cm (outer perimeter), using odometry.
// Sequence (right turns):
//   forward 0.30m -> turn right 90deg -> forward 0.20m -> turn right 90deg -> repeat
// No backlash compensation.
// Convention: right turn is negative theta (matches OdomDiff2W_Basic).

// Arduino auto-prototype FIX (must be placed before all #includes)
enum class State : unsigned char;

#define ENCODERS3PI_IMPLEMENTATION
#include "Encoders3pi.h"
#include "WheelSpeedCtrl3pi.h"
#include "OdomDiff2W_Basic.h"
#include <math.h>

// ---------------- Geometry ----------------
static const float SIDE_LONG_M  = 0.30f;
static const float SIDE_SHORT_M = 0.20f;
static const float TURN_90_RAD  = (float)M_PI * 0.5f;

// ---------------- Odom config (your calibrated values) ----------------
static const float METERS_PER_COUNT_L = 0.000282149f;
static const float METERS_PER_COUNT_R = 0.000282149f;
static const float WHEELBASE_M        = 0.08275f;

// ---------------- Motion parameters ----------------
static const float FWD_AVG_CPS   = 220.0f;   // forward speed target
static const float TURN_DIFF_CPS = 200.0f;   // in-place turning strength (diff sign decides direction)

// stop detection (using windowed speed)
static const float    STOP_EPS_CPS = 5.0f;
static const uint16_t STOP_HOLD_MS = 120;
static const uint16_t STOP_MAX_MS  = 800;

// settle
static const uint16_t POST_TURN_SETTLE_MS = 150;

// loop safety
static const uint32_t EDGE_TIMEOUT_MS = 12000;

// ---------------- Simple shapers (copied concept from your main code) ----------------
struct AccelLimiter {
  float y = 0.0f;
  float accelUp = 220.0f;   // cps/s
  float accelDown = 350.0f; // cps/s
  uint32_t lastUs = 0;

  void reset(float v = 0.0f) { y = v; lastUs = micros(); }

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

static inline float clampCps(float x, float lim) {
  if (x > lim) return lim;
  if (x < -lim) return -lim;
  return x;
}

// avg/diff mapping (same as your main code)
//   tgtL = avg - diff
//   tgtR = avg + diff
static inline void shapedMotionUpdate(float avgDes, float diffDes) {
  const float avg  = limAvg.update(avgDes);
  const float diff = limDiff.update(diffDes);

  const float tgtL = clampCps(avg - diff, 250.0f);
  const float tgtR = clampCps(avg + diff, 250.0f);

  WheelSpeed3pi::setTargetCps(tgtL, tgtR);
  WheelSpeed3pi::update();
}

static inline void hardStopUpdate() {
  WheelSpeed3pi::setTargetCps(0.0f, 0.0f);
  WheelSpeed3pi::update();
}

// ---------------- TurnTracker (same idea as your main code) ----------------
struct TurnTracker {
  float accum = 0.0f;
  float prevTh = 0.0f;
  bool  started = false;

  void start(float th0) { accum = 0.0f; prevTh = th0; started = true; }

  float update(float thNow) {
    if (!started) { start(thNow); return accum; }
    const float d = atan2f(sinf(thNow - prevTh), cosf(thNow - prevTh));
    accum += d;
    prevTh = thNow;
    return accum;
  }
};

// ---------------- Odom ----------------
static OdomDiff2W_Basic odom;

// ---------------- FSM ----------------
enum class State : unsigned char {
  INIT = 0,
  FWD,
  STOP_BEFORE_TURN,
  TURN_RIGHT_90,
  POST_TURN_SETTLE
};

static State gState = State::INIT;

static uint8_t sideIdx = 0;     // 0: long, 1: short, 2: long, 3: short
static uint8_t lapCount = 0;    // number of completed rectangles

static float segLenM = SIDE_LONG_M;
static float segStartX = 0.0f, segStartY = 0.0f;

static uint32_t stateEnterMs = 0;
static uint32_t stopHoldStartMs = 0;

static TurnTracker turnTrk;

static inline float traveledSegM() {
  const auto p = odom.getPose();
  const float dx = p.x - segStartX;
  const float dy = p.y - segStartY;
  return sqrtf(dx*dx + dy*dy);
}

static inline void enterState(State s, uint32_t nowMs) {
  gState = s;
  stateEnterMs = nowMs;
  stopHoldStartMs = 0;

  if (s == State::FWD) {
    const auto p = odom.getPose();
    segStartX = p.x; segStartY = p.y;
    segLenM = (sideIdx % 2 == 0) ? SIDE_LONG_M : SIDE_SHORT_M;
  }

  if (s == State::TURN_RIGHT_90) {
    turnTrk.start(odom.getPose().th);
  }

  limAvg.reset(0.0f);
  limDiff.reset(0.0f);
}

static inline bool wheelsStopped(uint32_t nowMs) {
  const float vL = Encoders3pi::getSpeedLeftCPS();
  const float vR = Encoders3pi::getSpeedRightCPS();
  const bool stoppedNow = (fabsf(vL) < STOP_EPS_CPS) && (fabsf(vR) < STOP_EPS_CPS);

  if (stoppedNow) {
    if (stopHoldStartMs == 0) stopHoldStartMs = nowMs;
  } else {
    stopHoldStartMs = 0;
  }

  return (stopHoldStartMs != 0) && ((uint32_t)(nowMs - stopHoldStartMs) >= STOP_HOLD_MS);
}

void setup() {
  Serial.begin(115200);

  WheelSpeed3pi::Config cfg;
  cfg.ctrlDtMs = 20;
  cfg.speedWinMs = 100;
  cfg.leftForwardLevel  = LOW;
  cfg.rightForwardLevel = LOW;
  cfg.pidLimitEnableL = true; cfg.pidLimitAbsL = 60.0f;
  cfg.pidLimitEnableR = true; cfg.pidLimitAbsR = 60.0f;
  WheelSpeed3pi::begin(cfg);

  OdomDiff2W_Basic::Config ocfg;
  ocfg.metersPerCountL = METERS_PER_COUNT_L;
  ocfg.metersPerCountR = METERS_PER_COUNT_R;
  ocfg.wheelBaseM      = WHEELBASE_M;
  odom.begin(ocfg, true);
  odom.resetAll(0.0f, 0.0f, 0.0f);

  limAvg.accelUp = 220.0f;   limAvg.accelDown = 350.0f;
  limDiff.accelUp = 1200.0f; limDiff.accelDown = 1200.0f;
  limAvg.reset(0.0f);
  limDiff.reset(0.0f);

  sideIdx = 0;
  lapCount = 0;
  enterState(State::FWD, millis());
}

void loop() {
  const uint32_t nowMs = millis();

  odom.update();

  switch (gState) {
    case State::INIT: {
      enterState(State::FWD, nowMs);
    } break;

    case State::FWD: {
      shapedMotionUpdate(FWD_AVG_CPS, 0.0f);

      if (traveledSegM() >= segLenM) {
        enterState(State::STOP_BEFORE_TURN, nowMs);
      }

      if ((uint32_t)(nowMs - stateEnterMs) > EDGE_TIMEOUT_MS) {
        enterState(State::STOP_BEFORE_TURN, nowMs);
      }
    } break;

    case State::STOP_BEFORE_TURN: {
      hardStopUpdate();

      const bool maxStopHit = ((uint32_t)(nowMs - stateEnterMs) >= STOP_MAX_MS);
      if (wheelsStopped(nowMs) || maxStopHit) {
        enterState(State::TURN_RIGHT_90, nowMs);
      }
    } break;

    case State::TURN_RIGHT_90: {
      shapedMotionUpdate(0.0f, -fabsf(TURN_DIFF_CPS));

      const float acc = turnTrk.update(odom.getPose().th);
      if (fabsf(acc) >= TURN_90_RAD) {
        enterState(State::POST_TURN_SETTLE, nowMs);
      }

      if ((uint32_t)(nowMs - stateEnterMs) > EDGE_TIMEOUT_MS) {
        enterState(State::POST_TURN_SETTLE, nowMs);
      }
    } break;

    case State::POST_TURN_SETTLE: {
      hardStopUpdate();

      if ((uint32_t)(nowMs - stateEnterMs) >= POST_TURN_SETTLE_MS) {
        sideIdx++;
        if (sideIdx >= 4) {
          sideIdx = 0;
          lapCount++;

          if (lapCount >= 2) {
            while (1) {
              hardStopUpdate();
              const auto p = odom.getPose();
              Serial.print("DONE, x="); Serial.print(p.x, 3);
              Serial.print(", y="); Serial.print(p.y, 3);
              Serial.print(", th="); Serial.println(p.th, 3);
              delay(500);
            }
          }
        }
        enterState(State::FWD, nowMs);
      }
    } break;
  }

  static uint32_t lastPrint = 0;
  if ((uint32_t)(nowMs - lastPrint) >= 200) {
    lastPrint = nowMs;
    const auto p = odom.getPose();
    Serial.print("t="); Serial.print(nowMs);
    Serial.print(", lap="); Serial.print(lapCount);
    Serial.print(", side="); Serial.print(sideIdx);
    Serial.print(", x="); Serial.print(p.x, 3);
    Serial.print(", y="); Serial.print(p.y, 3);
    Serial.print(", th="); Serial.println(p.th, 3);
  }
}
