// ===== TEST.ino =====
// Manual backlash measurement using serial-triggered single-shot tests.
//
// Workflow:
//  - Send 'a' to ARM one measurement.
//  - You have ARM_DELAY_MS time to get ready.
//  - The code preloads the gears, stops, waits, then reverses slowly and starts counting.
//  - The instant the wheel STARTS moving, send 's'.
//  - The code records the delta encoder counts and prints running mean/min/max.
//  - Returns to idle and waits for the next 'a'.
//
// Test types cycle automatically:
//   0: LEFT  + -> -
//   1: LEFT  - -> +
//   2: RIGHT + -> -
//   3: RIGHT - -> +
//
// Notes:
//  - Best practice: lift the robot so the wheels spin freely (ground friction inflates values).
//  - The encoder is on the motor side, so YOU must mark the moment the wheel starts moving.

#define ENCODERS3PI_IMPLEMENTATION
#include "Encoders3pi.h"
#include "WheelSpeedCtrl3pi.h"
#include <math.h>

using namespace WheelSpeed3pi;

// ---------------- Speeds (CPS) ----------------
static const float PRELOAD_CPS = 120.0f; // seat the gears
static const float MEASURE_CPS = 40.0f;  // slow reverse while you observe

// ---------------- Timing ----------------
static const uint32_t START_DELAY_MS = 1500;
static const uint32_t PRELOAD_MS     = 400;
static const uint32_t STOP_MS        = 250;
static const uint32_t ARM_DELAY_MS   = 1500; // your "get ready" time
static const uint32_t TIMEOUT_MS     = 7000; // timeout if you never press 's'

// ---------------- Helpers ----------------
static inline int32_t i32abs(int32_t x) { return (x < 0) ? -x : x; }

static float tgtL = 0.0f, tgtR = 0.0f;
static void setTarget(float l, float r) {
  tgtL = l; tgtR = r;
  WheelSpeed3pi::setTargetCps(l, r);
}
static void stopMotors() { setTarget(0.0f, 0.0f); }

static int32_t readWheelCount(bool isLeft) {
  return (int32_t)(isLeft ? Encoders3pi::getCountsLeft() : Encoders3pi::getCountsRight());
}

// ---------------- Simple state machine (uint8_t) ----------------
static const uint8_t ST_WAITBOOT  = 0;
static const uint8_t ST_IDLE      = 1;
static const uint8_t ST_PRELOAD   = 2;
static const uint8_t ST_STOP1     = 3;
static const uint8_t ST_ARM_DELAY = 4;
static const uint8_t ST_MEASURE   = 5;
static const uint8_t ST_STOP2     = 6;

static uint8_t st = ST_WAITBOOT;
static uint32_t tBoot_ms = 0;
static uint32_t stepStart_ms = 0;

// Current single-shot config (derived from testIdx)
static uint8_t  testIdx = 0;   // cycles 0..3
static bool     isLeft = true; // which wheel
static int8_t   preDir = +1;   // preload direction (+1/-1)
static int8_t   measDir = -1;  // measure direction (+1/-1)

static int32_t  baseCnt = 0;
static uint32_t measureStart_ms = 0;

// ---------------- Stats (separate per wheel) ----------------
static int32_t  sumL = 0, sumR = 0;
static uint16_t nL = 0, nR = 0;
static int16_t  minL = 32767, maxL = 0;
static int16_t  minR = 32767, maxR = 0;

static void resetStats() {
  sumL = sumR = 0;
  nL = nR = 0;
  minL = minR = 32767;
  maxL = maxR = 0;
}

static void printTestName(uint8_t idx) {
  // 0: L +->-, 1: L -->+, 2: R +->-, 3: R -->+
  if (idx == 0) Serial.print(F("LEFT + -> -"));
  else if (idx == 1) Serial.print(F("LEFT - -> +"));
  else if (idx == 2) Serial.print(F("RIGHT + -> -"));
  else               Serial.print(F("RIGHT - -> +"));
}

static void setupTestFromIdx(uint8_t idx) {
  // Derive wheel + directions from idx
  if (idx < 2) isLeft = true;
  else         isLeft = false;

  if ((idx % 2) == 0) { // + -> -
    preDir  = +1;
    measDir = -1;
  } else {              // - -> +
    preDir  = -1;
    measDir = +1;
  }
}

static void beginPreload() {
  // Preload only one wheel; keep the other at 0
  const float cps = (float)preDir * PRELOAD_CPS;
  if (isLeft) setTarget(cps, 0.0f);
  else        setTarget(0.0f, cps);
  stepStart_ms = millis();
}

static void beginStop() {
  stopMotors();
  stepStart_ms = millis();
}

static void beginMeasure() {
  const float cps = (float)measDir * MEASURE_CPS;
  if (isLeft) setTarget(cps, 0.0f);
  else        setTarget(0.0f, cps);

  baseCnt = readWheelCount(isLeft);
  measureStart_ms = millis();
  stepStart_ms = millis();
}

static void recordOne(int16_t v) {
  if (isLeft) {
    sumL += v; nL++;
    if (v < minL) minL = v;
    if (v > maxL) maxL = v;
  } else {
    sumR += v; nR++;
    if (v < minR) minR = v;
    if (v > maxR) maxR = v;
  }
}

static void printStats() {
  // Print running stats for both wheels
  Serial.println(F("---- Running stats ----"));
  if (nL > 0) {
    Serial.print(F("LEFT  mean="));
    Serial.print((float)sumL / (float)nL, 2);
    Serial.print(F(" counts, n=")); Serial.print(nL);
    Serial.print(F(", min=")); Serial.print(minL);
    Serial.print(F(", max=")); Serial.println(maxL);
  } else {
    Serial.println(F("LEFT  n=0"));
  }

  if (nR > 0) {
    Serial.print(F("RIGHT mean="));
    Serial.print((float)sumR / (float)nR, 2);
    Serial.print(F(" counts, n=")); Serial.print(nR);
    Serial.print(F(", min=")); Serial.print(minR);
    Serial.print(F(", max=")); Serial.println(maxR);
  } else {
    Serial.println(F("RIGHT n=0"));
  }
  Serial.println(F("-----------------------"));
}

static void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  a : arm ONE measurement (auto-cycles test type)"));
  Serial.println(F("  s : mark wheel-start moment during measuring"));
  Serial.println(F("  r : reset stats"));
  Serial.println(F("  ? : help"));
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  Config cfg;
  cfg.ctrlDtMs   = 20;
  cfg.speedWinMs = 100;
  cfg.stopEpsCps = 0.0f;
  WheelSpeed3pi::begin(cfg);

  Encoders3pi::resetCounts();
  resetStats();

  tBoot_ms = millis();

  Serial.println(F("=== BACKLASH SINGLE-SHOT MEASURE ==="));
  Serial.println(F("Best: lift robot so wheels spin freely."));
  Serial.println(F("Press 'a' to arm one test; after reversal starts, press 's' exactly when wheel starts moving."));
  printHelp();
}

void loop() {
  WheelSpeed3pi::update();
  const uint32_t now = millis();

  char key = 0;
  if (Serial.available()) key = (char)Serial.read();

  if (key == '?') printHelp();
  if (key == 'r' || key == 'R') {
    resetStats();
    Serial.println(F("Stats reset."));
  }

  switch (st) {
    case ST_WAITBOOT: {
      stopMotors();
      if (now - tBoot_ms >= START_DELAY_MS) {
        st = ST_IDLE;
        Serial.println(F("Ready. Send 'a' to start one measurement."));
      }
    } break;

    case ST_IDLE: {
      stopMotors();

      if (key == 'a' || key == 'A') {
        setupTestFromIdx(testIdx);

        Serial.print(F("\nARMED: ")); printTestName(testIdx); Serial.println();
        Serial.print(F("Preload ")); Serial.print(preDir > 0 ? F("+") : F("-"));
        Serial.print(F(" for ")); Serial.print(PRELOAD_MS); Serial.println(F("ms, then stop, then wait ARM_DELAY, then reverse & count."));
        Serial.print(F("You will have ")); Serial.print(ARM_DELAY_MS);
        Serial.println(F("ms to prepare. After reverse begins, press 's' the instant wheel starts moving."));

        beginPreload();
        st = ST_PRELOAD;
      }
    } break;

    case ST_PRELOAD: {
      if (now - stepStart_ms >= PRELOAD_MS) {
        beginStop();
        st = ST_STOP1;
      }
    } break;

    case ST_STOP1: {
      if (now - stepStart_ms >= STOP_MS) {
        // User preparation delay
        stopMotors();
        stepStart_ms = now;
        st = ST_ARM_DELAY;
      }
    } break;

    case ST_ARM_DELAY: {
      stopMotors();
      if (now - stepStart_ms >= ARM_DELAY_MS) {
        Serial.print(F("MEASURING NOW: ")); printTestName(testIdx);
        Serial.println(F("  <-- press 's' when wheel starts moving"));
        beginMeasure();
        st = ST_MEASURE;
      }
    } break;

    case ST_MEASURE: {
      // Only accept 's' in this state
      if (key == 's' || key == 'S') {
        const int16_t v = (int16_t)i32abs(readWheelCount(isLeft) - baseCnt);

        Serial.print(F("RESULT: ")); printTestName(testIdx);
        Serial.print(F(" = ")); Serial.print(v); Serial.println(F(" counts"));

        recordOne(v);
        printStats();

        beginStop();
        st = ST_STOP2;
      } else if (now - measureStart_ms >= TIMEOUT_MS) {
        Serial.print(F("TIMEOUT: ")); printTestName(testIdx);
        Serial.println(F(" (no 's' received). Recording ignored (not added)."));

        beginStop();
        st = ST_STOP2;
      }
    } break;

    case ST_STOP2: {
      if (now - stepStart_ms >= STOP_MS) {
        // Advance to next test config
        testIdx = (uint8_t)((testIdx + 1) & 0x03);

        Serial.print(F("Next test will be: "));
        printTestName(testIdx);
        Serial.println();
        Serial.println(F("Send 'a' when ready."));

        st = ST_IDLE;
      }
    } break;
  }
}

