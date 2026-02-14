// ===== WheelBaseCalib_Right360.ino =====
// Calibrate effective wheelbase by rotating right to -360deg using encoder odometry.
// Convention: right turn is negative theta.
// No extra corrections. You manually preload backlash before start.

#define ENCODERS3PI_IMPLEMENTATION
#include "Encoders3pi.h"
#include "WheelSpeedCtrl3pi.h"
#include <math.h>

using namespace WheelSpeed3pi;

// --------- Use your calibrated meters-per-count (from straight test) ----------
static const float METERS_PER_COUNT_L = 0.000282149f;
static const float METERS_PER_COUNT_R = 0.000282149f;

// --------- Wheelbase guess (you will tweak this) ----------
static const float WHEELBASE_GUESS_M = 0.085f;   // 8.5cm initial guess

// ---------------- User knobs ----------------
static const float    TURN_CPS          = 100.0f;  // in-place right turn speed (CPS); keep moderate to reduce slip
static const uint32_t START_DELAY_MS    = 1500;    // time for you to preload backlash and place robot at the start pose

// "Motors are actually stopped" criterion (based on encoder speed CPS)
static const float    STOP_DETECT_CPS   = 2.0f;    // consider "almost stopped" when |speed| < this
static const uint32_t STOP_STABLE_MS    = 300;     // must stay below threshold continuously for this long to latch

static const uint32_t PRINT_PERIOD_MS   = 200;
// -------------------------------------------

struct OdomTurn {
  int16_t lastRawL = 0, lastRawR = 0;
  int32_t accL = 0, accR = 0;   // wrap-safe accumulated counts
  float th = 0.0f;              // integrated yaw (rad), right negative

  void reset() {
    Encoders3pi::resetCounts();
    lastRawL = Encoders3pi::getCountsLeft();
    lastRawR = Encoders3pi::getCountsRight();
    accL = accR = 0;
    th = 0.0f;
  }

  void update() {
    const int16_t rawL = Encoders3pi::getCountsLeft();
    const int16_t rawR = Encoders3pi::getCountsRight();

    const int16_t dL = (int16_t)(rawL - lastRawL);
    const int16_t dR = (int16_t)(rawR - lastRawR);

    lastRawL = rawL;
    lastRawR = rawR;

    accL += (int32_t)dL;
    accR += (int32_t)dR;

    const float dSL = (float)dL * METERS_PER_COUNT_L;
    const float dSR = (float)dR * METERS_PER_COUNT_R;

    const float dTh = (WHEELBASE_GUESS_M != 0.0f) ? ((dSR - dSL) / WHEELBASE_GUESS_M) : 0.0f;
    th += dTh;
  }

  float totalSL() const { return (float)accL * METERS_PER_COUNT_L; }
  float totalSR() const { return (float)accR * METERS_PER_COUNT_R; }
};

enum class Phase : uint8_t { WAIT, RUN, STOPPING, LOCKED };
static Phase phase = Phase::WAIT;

static OdomTurn odom;

static const float GOAL_TH = -2.0f * (float)M_PI;   // right 360 deg

static uint32_t tBoot_ms = 0;
static uint32_t tStopStableStart_ms = 0;

static bool locked = false;
static uint32_t tLocked_ms = 0;
static float th_locked = 0.0f;
static int32_t accL_locked = 0, accR_locked = 0;

static uint32_t lastPrint_ms = 0;

void setup() {
  Serial.begin(115200);

  Config cfg;
  cfg.ctrlDtMs   = 20;
  cfg.speedWinMs = 100;
  cfg.stopEpsCps = 0.0f;
  WheelSpeed3pi::begin(cfg);

  odom.reset();
  tBoot_ms = millis();
  phase = Phase::WAIT;

  Serial.println("=== WheelBaseCalib: Right turn to -360deg (theta negative) ===");
  Serial.println("t_ms,phase,th_rad,accL,accR,SL_m,SR_m,vL_cps,vR_cps,locked_t_ms,th_locked,accL_locked,accR_locked");
  Serial.print("metersPerCount="); Serial.print(METERS_PER_COUNT_L, 9);
  Serial.print(" wheelBaseGuess="); Serial.println(WHEELBASE_GUESS_M, 6);
}

void loop() {
  WheelSpeed3pi::update();
  odom.update();

  const uint32_t now = millis();
  const float vL = Encoders3pi::getSpeedLeftCPS();
  const float vR = Encoders3pi::getSpeedRightCPS();

  switch (phase) {
    case Phase::WAIT: {
      WheelSpeed3pi::setTargetCps(0.0f, 0.0f);
      if (now - tBoot_ms >= START_DELAY_MS) {
        // After you manually preload backlash and place the robot: reset again and start the actual measurement
        odom.reset();
        locked = false;
        tStopStableStart_ms = 0;
        phase = Phase::RUN;

        // In-place right turn: left wheel +, right wheel - -> (dSR - dSL) < 0 -> theta becomes negative
        WheelSpeed3pi::setTargetCps(+TURN_CPS, -TURN_CPS);
      }
    } break;

    case Phase::RUN: {
      WheelSpeed3pi::setTargetCps(+TURN_CPS, -TURN_CPS);
      if (odom.th <= GOAL_TH) {
        WheelSpeed3pi::setTargetCps(0.0f, 0.0f);
        tStopStableStart_ms = 0;
        phase = Phase::STOPPING;
      }
    } break;

    case Phase::STOPPING: {
      WheelSpeed3pi::setTargetCps(0.0f, 0.0f);

      const bool stoppedNow = (fabsf(vL) < STOP_DETECT_CPS) && (fabsf(vR) < STOP_DETECT_CPS);
      if (stoppedNow) {
        if (tStopStableStart_ms == 0) tStopStableStart_ms = now;
        if ((now - tStopStableStart_ms) >= STOP_STABLE_MS) {
          locked = true;
          tLocked_ms = now;
          th_locked = odom.th;
          accL_locked = odom.accL;
          accR_locked = odom.accR;
          phase = Phase::LOCKED;
        }
      } else {
        tStopStableStart_ms = 0;
      }
    } break;

    case Phase::LOCKED: {
      WheelSpeed3pi::setTargetCps(0.0f, 0.0f);
    } break;
  }

  // Print forever
  if (now - lastPrint_ms >= PRINT_PERIOD_MS) {
    lastPrint_ms = now;

    Serial.print(now);
    Serial.print(',');
    Serial.print((phase == Phase::WAIT) ? "WAIT" :
                 (phase == Phase::RUN) ? "RUN" :
                 (phase == Phase::STOPPING) ? "STOPPING" : "LOCKED");
    Serial.print(',');
    Serial.print(odom.th, 6);
    Serial.print(',');
    Serial.print(odom.accL);
    Serial.print(',');
    Serial.print(odom.accR);
    Serial.print(',');
    Serial.print(odom.totalSL(), 6);
    Serial.print(',');
    Serial.print(odom.totalSR(), 6);
    Serial.print(',');
    Serial.print(vL, 2);
    Serial.print(',');
    Serial.print(vR, 2);
    Serial.print(',');
    Serial.print(locked ? (int32_t)tLocked_ms : (int32_t)0);
    Serial.print(',');
    Serial.print(locked ? th_locked : 0.0f, 6);
    Serial.print(',');
    Serial.print(locked ? accL_locked : (int32_t)0);
    Serial.print(',');
    Serial.print(locked ? accR_locked : (int32_t)0);
    Serial.println();
  }
}

