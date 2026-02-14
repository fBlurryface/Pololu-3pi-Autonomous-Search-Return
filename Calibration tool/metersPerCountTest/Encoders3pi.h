#pragma once
#include <Arduino.h>

#if !defined(__AVR_ATmega32U4__)
#error "Encoders3pi.h is intended for ATmega32U4 (e.g., Pololu 3pi+ 32U4)."
#endif

class Encoders3pi {
public:
  // Initialize encoder hardware and interrupts (optional: can be omitted; first read/speed call will auto-begin)
  static void begin();

  // Read accumulated left/right counts (int16_t wraps around)
  static int16_t getCountsLeft();
  static int16_t getCountsRight();

  // Zero counts (reset odometry origin)
  static void resetCounts();      // zero both left and right
  static void resetCountsLeft();
  static void resetCountsRight();

  // ---------------- Windowed speed API (micros()-based, units: counts/s) ----------------
  // Usage:
  //   setup(): Encoders3pi::speedBegin(100);  // 100ms window
  //   loop():  Encoders3pi::speedUpdate();    // call periodically (recommended once every 5~20ms)
  //   Read:    getSpeedLeftCPS()/getSpeedRightCPS()
  static void  speedBegin(uint16_t window_ms = 100);
  static void  speedUpdate();
  static float getSpeedLeftCPS();
  static float getSpeedRightCPS();

private:
  static bool inited_;
};

// ---------------- Implementation (compile exactly once) ----------------
// Only in ONE file write:
//   #define ENCODERS3PI_IMPLEMENTATION
//   #include "Encoders3pi.h"
// Other files only include; do not define.
#ifdef ENCODERS3PI_IMPLEMENTATION

#include <avr/io.h>
#include <avr/interrupt.h>

bool Encoders3pi::inited_ = false;

// 3pi+ 32U4 encoder signals: XOR + B; A = XOR ^ B
// Left : XOR -> D8  (PB4 / PCINT4) ; B -> PE2 (no Arduino pin number)
// Right: XOR -> D7  (PE6 / INT6)   ; B -> D23 (PF0)
static inline bool readLeftXor()  { return (PINB & _BV(4)); }   // PB4
static inline bool readLeftB()    { return (PINE & _BV(2)); }   // PE2
static inline bool readRightXor() { return (PINE & _BV(6)); }   // PE6 (INT6)
static inline bool readRightB()   { return (PINF & _BV(0)); }   // PF0 (D23)

static volatile bool lastLA, lastLB;
static volatile bool lastRA, lastRB;

// Use uint16_t to avoid undefined behavior from signed overflow; cast to int16_t at the API boundary
static volatile uint16_t cntL, cntR;

// Atomic read of both counts (disable interrupts once, read both)
static inline void readCountsAtomic(int16_t &outL, int16_t &outR) {
  uint8_t oldSREG = SREG;
  cli();
  outL = (int16_t)cntL;
  outR = (int16_t)cntR;
  SREG = oldSREG;
}

// Left: pin-change interrupt on PB4 (PCINT4)
ISR(PCINT0_vect) {
  bool newB = readLeftB();
  bool newA = readLeftXor() ^ newB;

  cntL += (newA ^ lastLB) - (lastLA ^ newB);

  lastLA = newA;
  lastLB = newB;
}

// Right: external interrupt INT6 on PE6
ISR(INT6_vect) {
  bool newB = readRightB();
  bool newA = readRightXor() ^ newB;

  cntR += (newA ^ lastRB) - (lastRA ^ newB);

  lastRA = newA;
  lastRB = newB;
}

void Encoders3pi::begin() {
  if (inited_) return;
  inited_ = true;

  // Configure input pullups (XOR / B)
  pinMode(8, INPUT_PULLUP);    // left XOR  (PB4)
  pinMode(7, INPUT_PULLUP);    // right XOR (PE6/INT6)
  pinMode(23, INPUT_PULLUP);   // right B   (PF0)

  // PE2 (left B) has no Arduino pin: manually set as input with pullup
  DDRE  &= ~_BV(2);
  PORTE |=  _BV(2);

  uint8_t oldSREG = SREG;
  cli();

  // Initialize state
  lastLB = readLeftB();
  lastLA = readLeftXor() ^ lastLB;
  cntL = 0;

  lastRB = readRightB();
  lastRA = readRightXor() ^ lastRB;
  cntR = 0;

  // Left: enable PCINT0 group + select PCINT4 (PB4)
  PCICR  |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT4);
  PCIFR  |= _BV(PCIF0);   // clear pending

  // Right: INT6 triggers on any logical change (ISC61:ISC60 = 01)
  EIMSK &= ~_BV(INT6);
  EICRB = (EICRB & ~(_BV(ISC61) | _BV(ISC60))) | _BV(ISC60);
  EIFR  |= _BV(INTF6);
  EIMSK |= _BV(INT6);

  SREG = oldSREG;
}

int16_t Encoders3pi::getCountsLeft() {
  begin();
  int16_t L, R;
  readCountsAtomic(L, R);
  return L;
}

int16_t Encoders3pi::getCountsRight() {
  begin();
  int16_t L, R;
  readCountsAtomic(L, R);
  return R;
}

void Encoders3pi::resetCounts() {
  begin();
  uint8_t oldSREG = SREG;
  cli();
  cntL = 0;
  cntR = 0;
  SREG = oldSREG;

  // To avoid "speed blowing up right after resetting counts", also reset the speed window here
  speedBegin((uint16_t)(/*keep window*/ 0)); // 0 means keep the current window (handled inside speedBegin)
}

void Encoders3pi::resetCountsLeft() {
  begin();
  uint8_t oldSREG = SREG;
  cli();
  cntL = 0;
  SREG = oldSREG;

  // Reset speed in sync (avoid speed jump)
  speedBegin((uint16_t)0);
}

void Encoders3pi::resetCountsRight() {
  begin();
  uint8_t oldSREG = SREG;
  cli();
  cntR = 0;
  SREG = oldSREG;

  speedBegin((uint16_t)0);
}

// ---------------- Windowed speed (micros-based) ----------------
// Note: do not call speedUpdate()/getSpeed*() inside ISRs (use only in loop()).
// Speed unit: counts per second (CPS); the window is a moving average (suppresses jitter/discrete jumps).

static volatile bool     spInited    = false;
static volatile uint32_t spWindowUs  = 100000UL;   // default 100ms

static volatile int16_t  spLastL = 0;
static volatile int16_t  spLastR = 0;
static volatile uint32_t spLastT = 0;

#ifndef ENC3PI_SPEED_BUF
#define ENC3PI_SPEED_BUF 32
#endif

static int16_t  spDL[ENC3PI_SPEED_BUF];
static int16_t  spDR[ENC3PI_SPEED_BUF];
static uint32_t spDT[ENC3PI_SPEED_BUF];

static uint8_t  spHead = 0;   // next write position
static uint8_t  spUsed = 0;   // number of used samples (<= BUF)
static int32_t  spSumDL = 0;
static int32_t  spSumDR = 0;
static uint32_t spSumDT = 0;

static inline uint8_t spOldestIndex() {
  // oldest = head - used
  return (uint8_t)((spHead + ENC3PI_SPEED_BUF - spUsed) % ENC3PI_SPEED_BUF);
}

void Encoders3pi::speedBegin(uint16_t window_ms) {
  begin();

  // window_ms = 0 means "keep the existing window, only reset the window contents"
  uint32_t keepWindowUs = spWindowUs;

  if (window_ms != 0) {
    if (window_ms == 0) window_ms = 1;
    spWindowUs = (uint32_t)window_ms * 1000UL;
  } else {
    spWindowUs = keepWindowUs;
  }

  uint32_t now = micros();

  // Reset ring buffer
  spHead = 0;
  spUsed = 0;
  spSumDL = 0;
  spSumDR = 0;
  spSumDT = 0;

  for (uint8_t i = 0; i < ENC3PI_SPEED_BUF; i++) {
    spDL[i] = 0;
    spDR[i] = 0;
    spDT[i] = 0;
  }

  // Use current accumulated counts as the baseline (avoid "missing counts")
  int16_t curL, curR;
  readCountsAtomic(curL, curR);

  spLastL = curL;
  spLastR = curR;
  spLastT = now;

  spInited = true;
}

void Encoders3pi::speedUpdate() {
  begin();
  if (!spInited) {
    speedBegin(100);
  }

  uint32_t now = micros();
  uint32_t dt  = (uint32_t)(now - spLastT); // unsigned wrap-safe
  if (dt == 0) return;

  // If the main loop stalls for a long time, dt becomes large; clamp it
  if (dt > 2000000UL) dt = 2000000UL; // 2s

  int16_t curL, curR;
  readCountsAtomic(curL, curR);

  int16_t dL = (int16_t)(curL - spLastL);
  int16_t dR = (int16_t)(curR - spLastR);

  spLastL = curL;
  spLastR = curR;
  spLastT = now;

  // Overwrite into ring buffer
  if (spUsed == ENC3PI_SPEED_BUF) {
    // overwrite oldest at spHead
    spSumDL -= spDL[spHead];
    spSumDR -= spDR[spHead];
    spSumDT -= spDT[spHead];
  } else {
    spUsed++;
  }

  spDL[spHead] = dL;
  spDR[spHead] = dR;
  spDT[spHead] = dt;

  spSumDL += dL;
  spSumDR += dR;
  spSumDT += dt;

  spHead = (uint8_t)((spHead + 1) % ENC3PI_SPEED_BUF);

  // Trim old samples until they fit inside the window (moving average)
  while (spUsed > 1 && spSumDT > spWindowUs) {
    uint8_t oldest = spOldestIndex();
    spSumDL -= spDL[oldest];
    spSumDR -= spDR[oldest];
    spSumDT -= spDT[oldest];
    spUsed--;
  }
}

float Encoders3pi::getSpeedLeftCPS() {
  if (!spInited) speedBegin(100);

  // Atomic read (avoid speedUpdate modifying sums at the same time)
  uint8_t oldSREG = SREG;
  cli();
  int32_t  sum = spSumDL;
  uint32_t dt  = spSumDT;
  SREG = oldSREG;

  if (dt == 0) return 0.0f;
  return (float)sum * 1000000.0f / (float)dt;
}

float Encoders3pi::getSpeedRightCPS() {
  if (!spInited) speedBegin(100);

  uint8_t oldSREG = SREG;
  cli();
  int32_t  sum = spSumDR;
  uint32_t dt  = spSumDT;
  SREG = oldSREG;

  if (dt == 0) return 0.0f;
  return (float)sum * 1000000.0f / (float)dt;
}

#endif // ENCODERS3PI_IMPLEMENTATION
