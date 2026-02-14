// ============================================================
// LineSensorsRC full API demo (power-on auto run)
// Demonstrates ALL public APIs:
//   - init(calibrate_ms)
//   - update()  (non-blocking sampling; true when a new frame is ready)
//   - readRaw(uint16_t out[5])
//   - readNorm(uint16_t out[5])
// ============================================================

#include <Arduino.h>
#include "LineSensorsRC.h"

LineSensorsRC line;

// Buffers required by the API
static uint16_t rawv[LineSensorsRC::N];
static uint16_t normv[LineSensorsRC::N];

// Demo parameters (tweak if you want)
static const uint16_t CALIB_MS = 4000;     // non-blocking auto calibration duration
static const uint16_t PRINT_HZ = 20;       // print at most this rate (frames may be faster)
static const uint16_t EDGE_TH = 750;       // example "dark edge" threshold on norm (0..1000)
static const uint16_t WHITE_TH = 600;      // example "white-ish" threshold on norm

static uint32_t startMs = 0;
static uint32_t lastPrintMs = 0;

static inline bool isCalibPeriod(uint32_t nowMs) {
  return (uint32_t)(nowMs - startMs) < (uint32_t)CALIB_MS;
}

// Compute a simple weighted position using norm values (0..1000).
// Returns pos in [-2000..+2000] approximately (left negative, right positive).
// If all weights are ~0, returns 0.
static int32_t computeLinePosWeighted(const uint16_t n[5]) {
  // Sensor index -> position weight (arbitrary symmetric scale)
  static const int16_t w[5] = { -2000, -1000, 0, 1000, 2000 };

  int32_t num = 0;
  int32_t den = 0;
  for (uint8_t i = 0; i < 5; i++) {
    // Treat darker as stronger signal by using n[i] directly
    const int32_t wi = (int32_t)w[i];
    const int32_t si = (int32_t)n[i];
    num += wi * si;
    den += si;
  }
  if (den <= 0) return 0;
  return num / den;
}

static void printHeaderOnce() {
  Serial.println(
    "t_ms,phase,"
    "raw0,raw1,raw2,raw3,raw4,"
    "n0,n1,n2,n3,n4,"
    "edgeMask,allWhite,posW"
  );
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // 1) init() : turns on emitters, sets up pins, and starts non-blocking calibration
  // During calibration, move the robot over typical background/line/edge.
  line.init(CALIB_MS);

  startMs = millis();
  lastPrintMs = 0;

  printHeaderOnce();
}

void loop() {
  const uint32_t nowMs = millis();

  // 2) update() : advance the non-blocking charge/discharge state machine.
  // Returns true when a full frame is ready.
  if (!line.update()) {
    return;
  }

  // 3) readRaw() : reads the latest raw discharge times in microseconds (uint16_t)
  line.readRaw(rawv);

  // 4) readNorm() : reads the latest normalized values (0..1000)
  line.readNorm(normv);

  // Optional: reduce serial spam (print at most PRINT_HZ)
  const uint32_t printPeriodMs = (PRINT_HZ == 0) ? 0 : (1000UL / PRINT_HZ);
  if (printPeriodMs != 0 && (uint32_t)(nowMs - lastPrintMs) < printPeriodMs) {
    return;
  }
  lastPrintMs = nowMs;

  // Phase label: "CAL" during calibration window, else "RUN"
  const char* phase = isCalibPeriod(nowMs) ? "CAL" : "RUN";

  // Example logic using norm data
  // edgeMask: bit i is 1 if norm[i] >= EDGE_TH
  uint8_t edgeMask = 0;
  for (uint8_t i = 0; i < LineSensorsRC::N; i++) {
    if (normv[i] >= EDGE_TH) edgeMask |= (uint8_t)(1U << i);
  }

  // allWhite: all channels below WHITE_TH
  bool allWhite = true;
  for (uint8_t i = 0; i < LineSensorsRC::N; i++) {
    if (normv[i] >= WHITE_TH) { allWhite = false; break; }
  }

  // Weighted "line position"
  const int32_t posW = computeLinePosWeighted(normv);

  // CSV print
  Serial.print((uint32_t)(nowMs - startMs));
  Serial.print(',');
  Serial.print(phase);
  Serial.print(',');

  // raw 5
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print(rawv[i]);
    Serial.print(',');
  }

  // norm 5
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print(normv[i]);
    Serial.print(',');
  }

  Serial.print((unsigned int)edgeMask);
  Serial.print(',');
  Serial.print(allWhite ? 1 : 0);
  Serial.print(',');
  Serial.println(posW);
}
