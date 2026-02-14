// ===== StraightCountsCalib_StopLock.ino =====
// Run straight until fixed counts, command stop, wait until motor CPS ~ 0, then lock counts and print forever.

#define ENCODERS3PI_IMPLEMENTATION
#include "Encoders3pi.h"
#include "WheelSpeedCtrl3pi.h"

using namespace WheelSpeed3pi;

// ---------------- User knobs ----------------
static const int32_t  TARGET_COUNTS_AVG = 3000;   // enter stop procedure when the average of left/right accumulated counts reaches this value
static const float    TARGET_CPS        = 120.0f; // straight-line target wheel speed (CPS)
static const uint32_t START_DELAY_MS    = 1500;   // delay after power-on so you can preload backlash and place robot at the start pose

// "Motors are actually stopped" criterion (based on encoder speed CPS)
static const float    STOP_DETECT_CPS   = 2.0f;   // consider "almost stopped" when |speed| < this
static const uint32_t STOP_STABLE_MS    = 300;    // must stay below threshold continuously for this long to latch counts

static const uint32_t PRINT_PERIOD_MS   = 200;    // serial print period
// -------------------------------------------

enum class Phase : uint8_t { WAIT, RUN, STOPPING, LOCKED };
static Phase phase = Phase::WAIT;

static int16_t lastRawL = 0, lastRawR = 0;
static int32_t accL = 0, accR = 0;          // wrap-safe accumulated counts

static uint32_t tBoot_ms = 0;
static uint32_t tStopCmd_ms = 0;
static uint32_t tStopStableStart_ms = 0;

static bool locked = false;
static int32_t accL_locked = 0, accR_locked = 0;
static uint32_t tLocked_ms = 0;

static uint32_t lastPrint_ms = 0;

static inline int32_t avgCounts(int32_t a, int32_t b) { return (a + b) / 2; }

static void resetAccumulators() {
  Encoders3pi::resetCounts();
  lastRawL = Encoders3pi::getCountsLeft();
  lastRawR = Encoders3pi::getCountsRight();
  accL = 0;
  accR = 0;
}

void setup() {
  Serial.begin(115200);

  Config cfg;            // use your default parameters (override here if needed)
  cfg.ctrlDtMs   = 20;
  cfg.speedWinMs = 100;
  cfg.stopEpsCps = 0.0f;
  WheelSpeed3pi::begin(cfg);

  resetAccumulators();

  tBoot_ms = millis();
  phase = Phase::WAIT;

  Serial.println("=== StraightCountsCalib (lock when motor stops) ===");
  Serial.println("t_ms,phase,accL,accR,avgAcc,rawL,rawR,vL_cps,vR_cps,locked_t_ms,accL_locked,accR_locked");
}

void loop() {
  WheelSpeed3pi::update();

  // ---- read raw counts, accumulate wrap-safe increments ----
  const int16_t rawL = Encoders3pi::getCountsLeft();
  const int16_t rawR = Encoders3pi::getCountsRight();
  const int16_t dL = (int16_t)(rawL - lastRawL);
  const int16_t dR = (int16_t)(rawR - lastRawR);
  lastRawL = rawL;
  lastRawR = rawR;
  accL += (int32_t)dL;
  accR += (int32_t)dR;

  const uint32_t now = millis();

  const float vL = Encoders3pi::getSpeedLeftCPS();
  const float vR = Encoders3pi::getSpeedRightCPS();

  // ---- state machine ----
  switch (phase) {
    case Phase::WAIT: {
      WheelSpeed3pi::setTargetCps(0.0f, 0.0f);
      if (now - tBoot_ms >= START_DELAY_MS) {
        // After you manually preload backlash and place the robot at the start, reset again for clean data
        resetAccumulators();
        locked = false;
        tStopStableStart_ms = 0;
        phase = Phase::RUN;
        WheelSpeed3pi::setTargetCps(TARGET_CPS, TARGET_CPS);
      }
    } break;

    case Phase::RUN: {
      WheelSpeed3pi::setTargetCps(TARGET_CPS, TARGET_CPS);
      const int32_t avgAcc = avgCounts(accL, accR);
      if (abs(avgAcc) >= TARGET_COUNTS_AVG) {
        // Issue stop command
        WheelSpeed3pi::setTargetCps(0.0f, 0.0f);
        tStopCmd_ms = now;
        tStopStableStart_ms = 0;
        phase = Phase::STOPPING;
      }
    } break;

    case Phase::STOPPING: {
      WheelSpeed3pi::setTargetCps(0.0f, 0.0f);

      // Decide "actually stopped" (CPS stays below threshold continuously for a while)
      const bool stoppedNow = (fabsf(vL) < STOP_DETECT_CPS) && (fabsf(vR) < STOP_DETECT_CPS);
      if (stoppedNow) {
        if (tStopStableStart_ms == 0) tStopStableStart_ms = now;
        if ((now - tStopStableStart_ms) >= STOP_STABLE_MS) {
          accL_locked = accL;
          accR_locked = accR;
          tLocked_ms = now;
          locked = true;
          phase = Phase::LOCKED;
        }
      } else {
        tStopStableStart_ms = 0;
      }
    } break;

    case Phase::LOCKED: {
      WheelSpeed3pi::setTargetCps(0.0f, 0.0f);
      // Do not update locked values anymore; just keep printing
    } break;
  }

  // ---- telemetry (forever) ----
  if (now - lastPrint_ms >= PRINT_PERIOD_MS) {
    lastPrint_ms = now;
    const int32_t avgAcc = avgCounts(accL, accR);

    Serial.print(now);
    Serial.print(',');
    Serial.print((phase == Phase::WAIT) ? "WAIT" :
                 (phase == Phase::RUN) ? "RUN" :
                 (phase == Phase::STOPPING) ? "STOPPING" : "LOCKED");
    Serial.print(',');
    Serial.print(accL);
    Serial.print(',');
    Serial.print(accR);
    Serial.print(',');
    Serial.print(avgAcc);
    Serial.print(',');
    Serial.print(rawL);
    Serial.print(',');
    Serial.print(rawR);
    Serial.print(',');
    Serial.print(vL, 2);
    Serial.print(',');
    Serial.print(vR, 2);
    Serial.print(',');
    Serial.print(locked ? (int32_t)tLocked_ms : (int32_t)0);
    Serial.print(',');
    Serial.print(locked ? accL_locked : (int32_t)0);
    Serial.print(',');
    Serial.print(locked ? accR_locked : (int32_t)0);
    Serial.println();
  }
}

