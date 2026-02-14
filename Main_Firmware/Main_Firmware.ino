// ============================================================
// Arduino auto-prototype FIX (must be placed before all #include)
// ============================================================
// Arduino may auto-generate function prototypes before it sees enum definitions.
// Forward-declaring these enums prevents build-order related compile errors.
enum class EdgeAction : unsigned char;
enum class RobotState : unsigned char;
enum class AvoidSource : unsigned char;

#include <Arduino.h>
#include <math.h>

#define ENCODERS3PI_IMPLEMENTATION
#include "Encoders3pi.h"

#define MAGNET_DETECTOR3PI_IMPLEMENTATION
#include "MagnetDetector3pi.h"

#include "WheelSpeedCtrl3pi.h"
#include "OdomDiff2W_Basic.h"
#include "LineSensorsRC.h"

// ============================================================
// 1) Config
// ============================================================

// ----- Init -----
static const uint32_t LINE_CALIB_MS = 8000UL;               // line sensor auto-calibration duration (non-blocking)
static const float    INIT_TURN_DEG = 450.0f;               // init spin angle (deg), used to expose sensors + set "east"
static const float    INIT_FWD_M    = 0.28f;                // init forward distance (m), used to place virtual boundary origin
static const uint16_t INIT_POST_TURN_SETTLE_MS = 500;       // settle after init spin (ms)

// ----- Explore settle -----
static const uint16_t EXPLORE_POST_TURN_SETTLE_MS = 150;    // settle after avoidance pivot (ms)

// ----- Line thresholds (0..1000) -----
// Convention: higher normalized value = "darker" (longer discharge time).
static const uint16_t EDGE_THRESHOLD_NORM = 750;
static const uint16_t WHITE_THRESHOLD_NORM =
  (EDGE_THRESHOLD_NORM >= 150) ? (EDGE_THRESHOLD_NORM - 150) : 0;

// ----- Edge behavior angles -----
static const float EDGE_TURN_DEG  = 75.0f;
static const float EDGE_UTURN_DEG = 180.0f;

// ----- Hard-stop detection (must stop before pivot) -----
static const float    STOP_EPS_CPS = 8.0f;                  // counts/s threshold treated as "stopped"
static const uint16_t STOP_HOLD_MS = 120;                   // must stay stopped for this long
static const uint16_t EDGE_STOP_MAX_MS = 800;               // fallback: pivot even if not fully stopped

// ----- Edge action min/max window -----
static const uint16_t EDGE_MIN_MS = 350;                    // minimum pivot duration
static const uint16_t EDGE_MAX_MS = 1200;                   // maximum pivot duration (escape hatch)

// ----- Trigger lockout -----
static const uint16_t EDGE_LOCK_MS = 450;                   // ignore edge triggers for a while after handling
static const uint16_t EDGE_CLEAR_WHITE_HOLD_MS = 300;       // for U-turn: require stable "all-white" before unlocking

// ----- Safety -----
static const uint16_t EDGE_ESCAPE_TIMEOUT_MS = 15000;       // global timeout for avoid routine
static const uint32_t TIME_LIMIT_MS = 120000;               // 0=disabled

// ----- Speed knobs (keep your values) -----
// avgCps: forward speed target; diffCps: in-place turn command magnitude.
static float initTurnDiffCps     = -200.0f;
static float initForwardAvgCps   = 300.0f;
static float exploreCruiseAvgCps = 300.0f;
static float exploreTurnDiffCps  = 200.0f;

// ============================================================
// 1.5) Virtual boundary config
// ============================================================
// Virtual boundary: 1D position along "east" axis, defined after init spin.
// The robot arms the boundary after moving forward enough, then triggers if it crosses back.
static const float VBOUND_S0_M = INIT_FWD_M;
static const float VBOUND_HYST_M = 0.010f;
static const float VBOUND_ARM_MARGIN_M = 0.030f;
static const uint16_t VBOUND_LOCK_MS = 900;

// ============================================================
// 2) Motion shaping: keep only "avg forward" and "diff turn"
// ============================================================
// Mapping:
//   tgtL = avg - diff
//   tgtR = avg + diff
// With this mapping: diff > 0 tends to turn left (CCW), diff < 0 tends to turn right (CW).
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

static inline void resetShapersToZero() {
  limAvg.reset(0.0f);
  limDiff.reset(0.0f);
}

// ============================================================
// 2.5) TurnTracker: accumulate angle robustly (wrap-safe)
// ============================================================

struct TurnTracker {
  float accum = 0.0f;
  float prevTh = 0.0f;
  bool  started = false;

  void start(float th0) {
    accum = 0.0f;
    prevTh = th0;
    started = true;
  }

  float update(float thNow) {
    if (!started) { start(thNow); return accum; }
    const float d = atan2f(sinf(thNow - prevTh), cosf(thNow - prevTh));
    accum += d;
    prevTh = thNow;
    return accum;
  }
};

static inline float deg2rad(float deg) { return deg * (float)M_PI / 180.0f; }
static inline float wrapToPi(float a) { return atan2f(sinf(a), cosf(a)); }

// ============================================================
// 3) Odometry + backlash
// ============================================================
// Backlash compensation is optional (counts can be 0 to disable).
// When wheel direction reverses, we "eat" the first BACKLASH_COUNTS_* counts before integrating.
OdomDiff2W_Basic odom;

const int16_t BACKLASH_COUNTS_L = 0;
const int16_t BACKLASH_COUNTS_R = 0;
const bool ASSUME_START_PRELOADED = true;

static int16_t pendBL_L = 0;
static int16_t pendBL_R = 0;
static int8_t  lastNZDirL = 0;
static int8_t  lastNZDirR = 0;

static inline int16_t i16min(int16_t a, int16_t b) { return (a < b) ? a : b; }

static inline int8_t dirFromTarget(float cps) {
  if (cps >  1.0f) return +1;
  if (cps < -1.0f) return -1;
  return 0;
}

// Trigger backlash "pending counts" when commanded direction changes (based on targets, not measured speed).
static inline void checkBacklashTrigger(float tgtL_cps, float tgtR_cps) {
  const int8_t dL = dirFromTarget(tgtL_cps);
  const int8_t dR = dirFromTarget(tgtR_cps);

  if (dL != 0) {
    if (lastNZDirL != 0 && dL != lastNZDirL) pendBL_L = BACKLASH_COUNTS_L;
    else if (lastNZDirL == 0 && !ASSUME_START_PRELOADED) pendBL_L = BACKLASH_COUNTS_L;
    lastNZDirL = dL;
  }

  if (dR != 0) {
    if (lastNZDirR != 0 && dR != lastNZDirR) pendBL_R = BACKLASH_COUNTS_R;
    else if (lastNZDirR == 0 && !ASSUME_START_PRELOADED) pendBL_R = BACKLASH_COUNTS_R;
    lastNZDirR = dR;
  }
}

// Read raw deltas, apply backlash compensation, then integrate corrected deltas into odom.
static inline void updateOdometryWithBacklash() {
  int16_t dL_raw = 0, dR_raw = 0;
  odom.readEncoderDeltas(&dL_raw, &dR_raw);

  int16_t dL_fixed = dL_raw;
  int16_t dR_fixed = dR_raw;

  if (pendBL_L > 0) {
    if (lastNZDirL > 0 && dL_raw > 0) {
      const int16_t eat = i16min(pendBL_L, dL_raw);
      dL_fixed -= eat; pendBL_L -= eat;
    } else if (lastNZDirL < 0 && dL_raw < 0) {
      const int16_t eat = i16min(pendBL_L, (int16_t)(-dL_raw));
      dL_fixed += eat; pendBL_L -= eat;
    }
  }

  if (pendBL_R > 0) {
    if (lastNZDirR > 0 && dR_raw > 0) {
      const int16_t eat = i16min(pendBL_R, dR_raw);
      dR_fixed -= eat; pendBL_R -= eat;
    } else if (lastNZDirR < 0 && dR_raw < 0) {
      const int16_t eat = i16min(pendBL_R, (int16_t)(-dR_raw));
      dR_fixed += eat; pendBL_R -= eat;
    }
  }

  odom.integrateDeltas(dL_fixed, dR_fixed);
}

// ============================================================
// 4) Line sensors + Magnet
// ============================================================

LineSensorsRC lineSensors;
static bool lineSensorsInited = false;
static uint16_t lineNorm[LineSensorsRC::N];

MagnetDetector3pi magnet;

// ============================================================
// 5) Motion helpers
// ============================================================

static inline void shapedMotionUpdate(float avgDes, float diffDes) {
  const float avg  = limAvg.update(avgDes);
  const float diff = limDiff.update(diffDes);

  const float tgtL = clampCps(avg - diff);
  const float tgtR = clampCps(avg + diff);

  checkBacklashTrigger(tgtL, tgtR);
  WheelSpeed3pi::setTargetCps(tgtL, tgtR);
  WheelSpeed3pi::update();
}

static inline void hardStopUpdate() {
  WheelSpeed3pi::setTargetCps(0.0f, 0.0f);
  WheelSpeed3pi::update();
}

// ============================================================
// 6) FSM + triggers
// ============================================================

enum class EdgeAction : unsigned char {
  None,
  TurnRight75,
  TurnLeft75,
  UTurn180,
  TurnToEastRandom
};

enum class AvoidSource : unsigned char { LineEdge, VirtualBoundary };

enum class RobotState : unsigned char {
  INIT_TURN_CALIB = 0,
  INIT_POST_TURN_SETTLE,
  INIT_DRIVE_FWD,

  EXPLORE_CRUISE,

  EDGE_HARD_STOP,
  EDGE_PIVOT_TURN,
  EDGE_POST_PIVOT_SETTLE,
  EDGE_CLEAR_DRIVE,

  // ===== Terminate cluster (Return Home) =====
  TERM_HARD_STOP,
  TERM_TURN_HOME,
  TERM_POST_TURN_SETTLE,
  TERM_DRIVE_HOME,
  TERM_HOLD
};

RobotState gState = RobotState::INIT_TURN_CALIB;

static uint32_t missionStartMs = 0;

// Init tracking
static uint32_t initStartMs = 0;
static bool  initRotationDone = false;
static uint32_t settleStartMs = 0;
static float initX0 = 0.0f, initY0 = 0.0f;
static bool  initDriveStarted = false;

// Turn trackers
static TurnTracker initTurnTrk;
static TurnTracker pivotTurnTrk;

// Current avoid action
static EdgeAction  edgeAction = EdgeAction::None;
static AvoidSource avoidSource = AvoidSource::LineEdge;

// lockouts
static uint32_t edgeLockUntilMs = 0;
static uint32_t vBoundLockUntilMs = 0;

// action timing
static uint32_t edgeEventStartMs = 0;

// hard stop tracking
static uint32_t stopEnterMs = 0;
static uint32_t stopHoldStartMs = 0;

// pivot tracking
static uint32_t pivotStartMs = 0;
static float pivotTargetRad = 0.0f;
static float pivotDiffCmd = 0.0f;

// all-white hold (only used by UTurn path)
static uint32_t allWhiteHoldStartMs = 0;

// --- edge logic ---
// Edge detection is evaluated ONLY when a new line sensor frame is available.
static bool edgeEvent = false;
static EdgeAction edgeEventAction = EdgeAction::None;
static bool edgeAllWhite = true;

// --- virtual boundary state ---
// eastRefTh/eastUx/eastUy are defined after INIT spin and used for 1D projection s = dot(east, pos).
static bool  eastRefValid = false;
static float eastRefTh = 0.0f;
static float eastUx = 1.0f, eastUy = 0.0f;
static bool  vBoundArmed = false;

static inline bool edgeLocked(uint32_t nowMs) { return nowMs < edgeLockUntilMs; }
static inline bool vBoundLocked(uint32_t nowMs) { return nowMs < vBoundLockUntilMs; }

static inline bool takeEdgeEvent(EdgeAction* outAction) {
  if (!edgeEvent) return false;
  *outAction = edgeEventAction;
  edgeEvent = false;
  return true;
}

static inline void processEdgeLogicOnNewFrame(bool allowTrigger) {
  const bool b0 = (lineNorm[0] >= EDGE_THRESHOLD_NORM);
  const bool b1 = (lineNorm[1] >= EDGE_THRESHOLD_NORM);
  const bool b2 = (lineNorm[2] >= EDGE_THRESHOLD_NORM);
  const bool b3 = (lineNorm[3] >= EDGE_THRESHOLD_NORM);
  const bool b4 = (lineNorm[4] >= EDGE_THRESHOLD_NORM);

  // "All white" means every sensor is below WHITE_THRESHOLD_NORM (safe background).
  bool allW = true;
  for (uint8_t i = 0; i < LineSensorsRC::N; i++) {
    if (lineNorm[i] >= WHITE_THRESHOLD_NORM) { allW = false; break; }
  }
  edgeAllWhite = allW;

  if (!allowTrigger) return;

  // Priority: all 5 dark -> U-turn; left-side dark -> turn right; right-side dark -> turn left.
  if (!edgeEvent) {
    if (b0 && b1 && b2 && b3 && b4) {
      edgeEvent = true; edgeEventAction = EdgeAction::UTurn180;
    } else if (b0 || b1) {
      edgeEvent = true; edgeEventAction = EdgeAction::TurnRight75;
    } else if (b3 || b4) {
      edgeEvent = true; edgeEventAction = EdgeAction::TurnLeft75;
    }
  }
}

static inline float eastProjS(const OdomDiff2W_Basic::Pose& p) {
  return eastUx * p.x + eastUy * p.y;
}

static inline void enterCruise(bool fromStop) {
  setPidClampCruise();
  if (fromStop) resetShapersToZero();
  gState = RobotState::EXPLORE_CRUISE;
}

static inline void startAvoidAction(EdgeAction a, AvoidSource src, uint32_t nowMs) {
  edgeAction = a;
  avoidSource = src;
  edgeEventStartMs = nowMs;

  // Always start with a hard stop, then pivot.
  hardStopUpdate();
  resetShapersToZero();

  stopEnterMs = nowMs;
  stopHoldStartMs = 0;

  gState = RobotState::EDGE_HARD_STOP;
}

static inline void beginPivot(uint32_t nowMs) {
  (void)nowMs;
  setPidClampEdge();
  resetShapersToZero();

  const float curTh = odom.getPose().th;

  // With tgtL=avg-diff and tgtR=avg+diff:
  //   pivotDiffCmd < 0 => right turn (CW)
  //   pivotDiffCmd > 0 => left turn  (CCW)
  if (edgeAction == EdgeAction::TurnRight75) {
    pivotTargetRad = deg2rad(EDGE_TURN_DEG);
    pivotDiffCmd   = -fabsf(exploreTurnDiffCps);
  } else if (edgeAction == EdgeAction::TurnLeft75) {
    pivotTargetRad = deg2rad(EDGE_TURN_DEG);
    pivotDiffCmd   = +fabsf(exploreTurnDiffCps);
  } else if (edgeAction == EdgeAction::UTurn180) {
    pivotTargetRad = deg2rad(EDGE_UTURN_DEG);
    pivotDiffCmd   = -fabsf(exploreTurnDiffCps);
  } else { // TurnToEastRandom
    // Turn toward eastRefTh with a random offset in [-90°, +90°], retrying to avoid tiny turns.
    float desired = eastRefTh;
    float delta = 0.0f;
    for (uint8_t k = 0; k < 3; k++) {
      const float u = (float)random(-1000, 1001) / 1000.0f;
      const float offset = u * (0.5f * (float)M_PI);
      desired = wrapToPi(eastRefTh + offset);
      delta = wrapToPi(desired - curTh);
      if (fabsf(delta) > deg2rad(10.0f)) break;
    }
    pivotTargetRad = fabsf(delta);
    pivotDiffCmd   = (delta >= 0.0f) ? +fabsf(exploreTurnDiffCps)
                                     : -fabsf(exploreTurnDiffCps);
  }

  pivotTurnTrk.start(curTh);
  pivotStartMs = millis();
  gState = RobotState::EDGE_PIVOT_TURN;
}

// ============================================================
// 6.5) Terminate cluster (Return Home)
// ============================================================
// Triggered by time limit or magnet detection during exploration.
// Sequence: stop -> turn to face origin -> drive straight to origin -> hold.
static bool terminateRequested = false;

static uint32_t termStopEnterMs = 0;
static uint32_t termStopHoldStartMs = 0;

static TurnTracker termTurnTrk;
static float termTurnTargetAbsRad = 0.0f;
static float termTurnDiffCmd = 0.0f;

static float termDriveTargetM = 0.0f;
static float termDriveStartX = 0.0f;
static float termDriveStartY = 0.0f;

static inline void startTerminate(uint32_t nowMs) {
  terminateRequested = true;

  // Disable all other triggers permanently once termination starts.
  edgeLockUntilMs = UINT32_MAX;
  vBoundLockUntilMs = UINT32_MAX;
  edgeEvent = false;
  vBoundArmed = false;

  hardStopUpdate();
  resetShapersToZero();

  termStopEnterMs = nowMs;
  termStopHoldStartMs = 0;

  gState = RobotState::TERM_HARD_STOP;
}

static inline void beginTurnHome(uint32_t nowMs) {
  (void)nowMs;
  const auto p = odom.getPose();

  // Desired heading points from current pose back to origin (0,0).
  const float desired = atan2f(-p.y, -p.x);
  const float delta   = wrapToPi(desired - p.th);

  termTurnTargetAbsRad = fabsf(delta);
  termTurnDiffCmd = (delta >= 0.0f) ? +fabsf(exploreTurnDiffCps)
                                    : -fabsf(exploreTurnDiffCps);

  setPidClampEdge();
  resetShapersToZero();

  termTurnTrk.start(p.th);
  gState = RobotState::TERM_TURN_HOME;
}

static inline void beginDriveHome(uint32_t nowMs) {
  (void)nowMs;
  const auto p = odom.getPose();
  termDriveTargetM = sqrtf(p.x * p.x + p.y * p.y);

  termDriveStartX = p.x;
  termDriveStartY = p.y;

  setPidClampCruise();
  resetShapersToZero();

  gState = RobotState::TERM_DRIVE_HOME;
}

// ============================================================
// 7) Setup / Loop
// ============================================================

void setup() {
  (void)magnet.begin();
  (void)magnet.calibrate(50, 20); // 50 samples * 20ms = 1000ms

  WheelSpeed3pi::Config cfg;
  cfg.ctrlDtMs = MP.ctrlDtMs;
  cfg.speedWinMs = MP.speedWinMs;
  cfg.leftForwardLevel  = MP.leftForwardLevel;
  cfg.rightForwardLevel = MP.rightForwardLevel;

  cfg.pidLimitEnableL = true; cfg.pidLimitAbsL = MP.cruisePidClampAbs;
  cfg.pidLimitEnableR = true; cfg.pidLimitAbsR = MP.cruisePidClampAbs;
  WheelSpeed3pi::begin(cfg);

  initShapers();
  setPidClampCruise();

  OdomDiff2W_Basic::Config odomCfg;
  odomCfg.metersPerCountL = 0.000282149f;
  odomCfg.metersPerCountR = 0.000282149f;
  odomCfg.wheelBaseM      = 0.08275f;
  odom.begin(odomCfg);

  randomSeed((uint32_t)micros());
  missionStartMs = millis();
}

void loop() {
  const uint32_t nowMs = millis();

  // ===== time-based terminate trigger (fires once) =====
  if (TIME_LIMIT_MS > 0 && !terminateRequested) {
    if ((uint32_t)(nowMs - missionStartMs) >= TIME_LIMIT_MS) {
      startTerminate(nowMs);
    }
  }

  // Odometry should update every loop (even while stopping/turning).
  updateOdometryWithBacklash();

  // [MAGNET] During exploration, detect magnet continuously; if detected, enter terminate cluster immediately.
  if (!terminateRequested && gState == RobotState::EXPLORE_CRUISE) {
    if (magnet.detect()) {
      startTerminate(nowMs);
      // After termination starts, do not process other triggers in this loop iteration.
    }
  }

  // ===== line update + edge logic (ONLY on new frame) =====
  bool lineNew = false;
  if (lineSensorsInited) {
    if (lineSensors.update()) {
      lineSensors.readNorm(lineNorm);
      lineNew = true;

      const bool allowEdgeTrigger = (gState == RobotState::EXPLORE_CRUISE) && !edgeLocked(nowMs) && !terminateRequested;
      processEdgeLogicOnNewFrame(allowEdgeTrigger);
    }
  }

  switch (gState) {

    // ---------------- INIT ----------------
    case RobotState::INIT_TURN_CALIB: {
      static bool entered = false;
      if (!entered) {
        entered = true;

        // Start non-blocking calibration and keep sampling for LINE_CALIB_MS.
        lineSensors.init((uint16_t)LINE_CALIB_MS);
        lineSensorsInited = true;

        initStartMs = nowMs;
        initRotationDone = false;

        setPidClampEdge();
        resetShapersToZero();
        initTurnTrk.start(odom.getPose().th);

        edgeEvent = false;
        edgeAllWhite = true;

        eastRefValid = false;
        vBoundArmed = false;
        vBoundLockUntilMs = UINT32_MAX;
      }

      const float targetRad = deg2rad(INIT_TURN_DEG);
      const float acc = initTurnTrk.update(odom.getPose().th);

      if (!initRotationDone) {
        if (fabsf(acc) < targetRad) {
          shapedMotionUpdate(0.0f, initTurnDiffCps);
        } else {
          initRotationDone = true;

          // Define "east" reference for virtual boundary projection.
          eastRefTh = odom.getPose().th;
          eastUx = cosf(eastRefTh);
          eastUy = sinf(eastRefTh);
          eastRefValid = true;

          vBoundArmed = false;
          vBoundLockUntilMs = nowMs + VBOUND_LOCK_MS;

          settleStartMs = nowMs;
          gState = RobotState::INIT_POST_TURN_SETTLE;
        }
      }
    } break;

    case RobotState::INIT_POST_TURN_SETTLE: {
      hardStopUpdate();

      const bool calibDone  = ((uint32_t)(nowMs - initStartMs) >= LINE_CALIB_MS);
      const bool settleDone = ((uint32_t)(nowMs - settleStartMs) >= INIT_POST_TURN_SETTLE_MS);

      if (calibDone && settleDone) {
        setPidClampCruise();
        resetShapersToZero();
        initDriveStarted = false;
        gState = RobotState::INIT_DRIVE_FWD;
      }
    } break;

    case RobotState::INIT_DRIVE_FWD: {
      if (!initDriveStarted) {
        initDriveStarted = true;
        const auto p0 = odom.getPose();
        initX0 = p0.x; initY0 = p0.y;

        setPidClampCruise();
        resetShapersToZero();
      }

      shapedMotionUpdate(initForwardAvgCps, 0.0f);

      const auto p = odom.getPose();
      const float dx = p.x - initX0;
      const float dy = p.y - initY0;
      const float dist = sqrtf(dx * dx + dy * dy);

      // After moving forward enough, unlock edge logic and enter cruise exploration.
      if (dist >= INIT_FWD_M) {
        edgeLockUntilMs = 0;
        enterCruise(true);
      }
    } break;

    // ---------------- EXPLORE ----------------
    case RobotState::EXPLORE_CRUISE: {
      // If magnet triggered termination earlier, this state will not run (gState already changed).
      shapedMotionUpdate(exploreCruiseAvgCps, 0.0f);

      EdgeAction act = EdgeAction::None;
      if (!terminateRequested && !edgeLocked(nowMs) && takeEdgeEvent(&act)) {
        startAvoidAction(act, AvoidSource::LineEdge, nowMs);
        break;
      }

      if (!terminateRequested && eastRefValid && !vBoundLocked(nowMs)) {
        const auto p = odom.getPose();
        const float s = eastProjS(p);

        // Arm after moving forward beyond s0 + margin.
        if (!vBoundArmed && s >= (VBOUND_S0_M + VBOUND_ARM_MARGIN_M)) {
          vBoundArmed = true;
        }

        // Trigger after crossing back past s0 - hysteresis.
        if (vBoundArmed && s < (VBOUND_S0_M - VBOUND_HYST_M)) {
          vBoundArmed = false;
          startAvoidAction(EdgeAction::TurnToEastRandom, AvoidSource::VirtualBoundary, nowMs);
          break;
        }
      }
    } break;

    // --------- Avoid: HARD STOP ----------
    case RobotState::EDGE_HARD_STOP: {
      hardStopUpdate();

      // Safety: do not get stuck forever.
      if ((uint32_t)(nowMs - edgeEventStartMs) >= EDGE_ESCAPE_TIMEOUT_MS) {
        settleStartMs = nowMs;
        gState = RobotState::EDGE_POST_PIVOT_SETTLE;
        break;
      }

      const bool maxStopTimeHit = ((uint32_t)(nowMs - stopEnterMs) >= EDGE_STOP_MAX_MS);

      const float vL = Encoders3pi::getSpeedLeftCPS();
      const float vR = Encoders3pi::getSpeedRightCPS();
      const bool stoppedNow = (fabsf(vL) < STOP_EPS_CPS) && (fabsf(vR) < STOP_EPS_CPS);

      if (stoppedNow) {
        if (stopHoldStartMs == 0) stopHoldStartMs = nowMs;
      } else {
        stopHoldStartMs = 0;
      }

      const bool holdOk = (stopHoldStartMs != 0) && ((uint32_t)(nowMs - stopHoldStartMs) >= STOP_HOLD_MS);

      if (holdOk || maxStopTimeHit) {
        beginPivot(nowMs);
      }
    } break;

    case RobotState::EDGE_PIVOT_TURN: {
      shapedMotionUpdate(0.0f, pivotDiffCmd);

      if ((uint32_t)(nowMs - edgeEventStartMs) >= EDGE_ESCAPE_TIMEOUT_MS) {
        settleStartMs = nowMs;
        gState = RobotState::EDGE_POST_PIVOT_SETTLE;
        break;
      }

      const uint32_t t = (uint32_t)(nowMs - pivotStartMs);
      const float acc = pivotTurnTrk.update(odom.getPose().th);
      const bool minAngOk = (fabsf(acc) >= pivotTargetRad);

      if (edgeAction == EdgeAction::UTurn180) {
        if (minAngOk) {
          settleStartMs = nowMs;
          gState = RobotState::EDGE_POST_PIVOT_SETTLE;
          allWhiteHoldStartMs = 0;
        }
      } else {
        const bool minTimeOk = (t >= EDGE_MIN_MS);
        const bool maxTimeHit = (t >= EDGE_MAX_MS);

        const bool needAllWhite = (avoidSource == AvoidSource::LineEdge);
        const bool okWhite = (!needAllWhite) ? true : edgeAllWhite;

        if ((minTimeOk && minAngOk && okWhite) || maxTimeHit) {
          settleStartMs = nowMs;
          gState = RobotState::EDGE_POST_PIVOT_SETTLE;
        }
      }
    } break;

    case RobotState::EDGE_POST_PIVOT_SETTLE: {
      hardStopUpdate();

      if ((uint32_t)(nowMs - settleStartMs) >= EXPLORE_POST_TURN_SETTLE_MS) {

        // U-turn path: do a clearing drive until all sensors see white stably.
        if (edgeAction == EdgeAction::UTurn180 && avoidSource == AvoidSource::LineEdge) {
          edgeLockUntilMs = UINT32_MAX;
          gState = RobotState::EDGE_CLEAR_DRIVE;
          break;
        }

        if (avoidSource == AvoidSource::LineEdge) {
          edgeLockUntilMs = nowMs + EDGE_LOCK_MS;
        } else {
          vBoundLockUntilMs = nowMs + VBOUND_LOCK_MS;
        }

        enterCruise(true);
      }
    } break;

    case RobotState::EDGE_CLEAR_DRIVE: {
      setPidClampCruise();
      shapedMotionUpdate(exploreCruiseAvgCps, 0.0f);

      if ((uint32_t)(nowMs - edgeEventStartMs) >= EDGE_ESCAPE_TIMEOUT_MS) {
        edgeLockUntilMs = nowMs + EDGE_LOCK_MS;
        enterCruise(true);
        break;
      }

      if (lineNew) {
        if (edgeAllWhite) {
          if (allWhiteHoldStartMs == 0) allWhiteHoldStartMs = nowMs;
          if ((uint32_t)(nowMs - allWhiteHoldStartMs) >= EDGE_CLEAR_WHITE_HOLD_MS) {
            edgeLockUntilMs = nowMs + EDGE_LOCK_MS;
            enterCruise(true);
          }
        } else {
          allWhiteHoldStartMs = 0;
        }
      }
    } break;

    // ========================================================
    // ================= Terminate (Return Home) ===============
    // ========================================================

    case RobotState::TERM_HARD_STOP: {
      hardStopUpdate();

      const bool maxStopTimeHit = ((uint32_t)(nowMs - termStopEnterMs) >= EDGE_STOP_MAX_MS);

      const float vL = Encoders3pi::getSpeedLeftCPS();
      const float vR = Encoders3pi::getSpeedRightCPS();
      const bool stoppedNow = (fabsf(vL) < STOP_EPS_CPS) && (fabsf(vR) < STOP_EPS_CPS);

      if (stoppedNow) {
        if (termStopHoldStartMs == 0) termStopHoldStartMs = nowMs;
      } else {
        termStopHoldStartMs = 0;
      }

      const bool holdOk = (termStopHoldStartMs != 0) && ((uint32_t)(nowMs - termStopHoldStartMs) >= STOP_HOLD_MS);

      if (holdOk || maxStopTimeHit) {
        beginTurnHome(nowMs);
      }
    } break;

    case RobotState::TERM_TURN_HOME: {
      shapedMotionUpdate(0.0f, termTurnDiffCmd);

      const float acc = termTurnTrk.update(odom.getPose().th);
      if (fabsf(acc) >= termTurnTargetAbsRad) {
        settleStartMs = nowMs;
        gState = RobotState::TERM_POST_TURN_SETTLE;
      }
    } break;

    case RobotState::TERM_POST_TURN_SETTLE: {
      hardStopUpdate();

      if ((uint32_t)(nowMs - settleStartMs) >= EXPLORE_POST_TURN_SETTLE_MS) {
        beginDriveHome(nowMs);
      }
    } break;

    case RobotState::TERM_DRIVE_HOME: {
      shapedMotionUpdate(exploreCruiseAvgCps, 0.0f);

      const auto p = odom.getPose();
      const float dx = p.x - termDriveStartX;
      const float dy = p.y - termDriveStartY;
      const float traveled = sqrtf(dx * dx + dy * dy);

      if (traveled >= termDriveTargetM) {
        gState = RobotState::TERM_HOLD;
      }
    } break;

    case RobotState::TERM_HOLD: {
      hardStopUpdate();
    } break;

    default: {
      hardStopUpdate();
    } break;
  }
}
