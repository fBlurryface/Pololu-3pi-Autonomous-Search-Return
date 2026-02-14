#pragma once
#include <Arduino.h>

#if !defined(__AVR_ATmega32U4__)
#error "Encoders3pi.h is intended for ATmega32U4 (e.g., Pololu 3pi+ 32U4)."
#endif

class Encoders3pi
{
public:
  // 在 setup() 调一次即可（也可以不调，首次读数时会自动 begin）
  static void begin();

  // 可选：整体翻转方向（如果你发现符号与期望相反）
  static void flip(bool enable);

  // 累计计数（int16_t 会回绕）
  static int16_t getCountsLeft();
  static int16_t getCountsRight();

  // 推荐：读增量并清零（用于算轮速）
  static int16_t getCountsAndResetLeft();
  static int16_t getCountsAndResetRight();

  // 读并清除错误标志（丢边沿/两路同时变等）
  static bool checkErrorLeft();
  static bool checkErrorRight();

private:
  static bool inited_;
  static bool flipped_;
};

// ---------------- Implementation (compile exactly once) ----------------
// 只在一个文件里写：
//   #define ENCODERS3PI_IMPLEMENTATION
//   #include "Encoders3pi.h"
// 其他文件只 include，不要 define。
#ifdef ENCODERS3PI_IMPLEMENTATION

#include <avr/io.h>
#include <avr/interrupt.h>

bool Encoders3pi::inited_  = false;
bool Encoders3pi::flipped_ = false;

// 3pi+ 32U4 编码器信号：XOR + B；A = XOR ^ B
// Left : XOR -> D8  (PB4 / PCINT4) ; B -> PE2 (无 Arduino pin number)
// Right: XOR -> D7  (PE6 / INT6)   ; B -> D23 (PF0)
static inline bool readLeftXor()  { return (PINB & _BV(4)); }   // PB4
static inline bool readLeftB()    { return (PINE & _BV(2)); }   // PE2
static inline bool readRightXor() { return (PINE & _BV(6)); }   // PE6 (INT6)
static inline bool readRightB()   { return (PINF & _BV(0)); }   // PF0 (D23)

static volatile bool lastLA, lastLB;
static volatile bool lastRA, lastRB;

static volatile bool errL, errR;

// 用 uint16_t 避免有符号溢出的未定义行为；对外再 cast 成 int16_t
static volatile uint16_t cntL, cntR;

// Left: pin-change interrupt on PB4 (PCINT4)
ISR(PCINT0_vect)
{
  bool newB = readLeftB();
  bool newA = readLeftXor() ^ newB;

  cntL += (newA ^ lastLB) - (lastLA ^ newB);

  // 两路同时变化通常意味着丢了边沿/时序不稳
  if ( (lastLA ^ newA) & (lastLB ^ newB) ) { errL = true; }

  lastLA = newA;
  lastLB = newB;
}

// Right: external interrupt INT6 on PE6
ISR(INT6_vect)
{
  bool newB = readRightB();
  bool newA = readRightXor() ^ newB;

  cntR += (newA ^ lastRB) - (lastRA ^ newB);

  if ( (lastRA ^ newA) & (lastRB ^ newB) ) { errR = true; }

  lastRA = newA;
  lastRB = newB;
}

void Encoders3pi::begin()
{
  if (inited_) return;
  inited_ = true;

  // 配置输入上拉（XOR / B）
  pinMode(8, INPUT_PULLUP);    // left XOR  (PB4)
  pinMode(7, INPUT_PULLUP);    // right XOR (PE6/INT6)
  pinMode(23, INPUT_PULLUP);   // right B   (PF0)

  // PE2 (left B) 没有 Arduino pin：手动设为输入上拉
  DDRE  &= ~_BV(2);
  PORTE |=  _BV(2);

  // 原子初始化 + 开中断（保留原来的全局中断状态）
  uint8_t oldSREG = SREG;
  cli();

  // 初始化状态，避免第一跳乱判
  lastLB = readLeftB();
  lastLA = readLeftXor() ^ lastLB;
  cntL = 0;
  errL = false;

  lastRB = readRightB();
  lastRA = readRightXor() ^ lastRB;
  cntR = 0;
  errR = false;

  // Left：开启 PCINT0 组 + 选择 PCINT4 (PB4)
  PCICR  |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT4);
  PCIFR  |= _BV(PCIF0);   // 清 pending

  // Right：INT6 任何电平变化触发（ISC61:ISC60 = 01）
  EIMSK &= ~_BV(INT6);
  EICRB = (EICRB & ~(_BV(ISC61) | _BV(ISC60))) | _BV(ISC60);
  EIFR  |= _BV(INTF6);
  EIMSK |= _BV(INT6);

  SREG = oldSREG;
}

void Encoders3pi::flip(bool enable)
{
  flipped_ = enable;
}

int16_t Encoders3pi::getCountsLeft()
{
  begin();
  uint8_t oldSREG = SREG;
  cli();
  int16_t c = (int16_t)cntL;
  SREG = oldSREG;

  if (flipped_) c = (int16_t)(-c);
  return c;
}

int16_t Encoders3pi::getCountsRight()
{
  begin();
  uint8_t oldSREG = SREG;
  cli();
  int16_t c = (int16_t)cntR;
  SREG = oldSREG;

  if (flipped_) c = (int16_t)(-c);
  return c;
}

int16_t Encoders3pi::getCountsAndResetLeft()
{
  begin();
  uint8_t oldSREG = SREG;
  cli();
  int16_t c = (int16_t)cntL;
  cntL = 0;
  SREG = oldSREG;

  if (flipped_) c = (int16_t)(-c);
  return c;
}

int16_t Encoders3pi::getCountsAndResetRight()
{
  begin();
  uint8_t oldSREG = SREG;
  cli();
  int16_t c = (int16_t)cntR;
  cntR = 0;
  SREG = oldSREG;

  if (flipped_) c = (int16_t)(-c);
  return c;
}

bool Encoders3pi::checkErrorLeft()
{
  begin();
  uint8_t oldSREG = SREG;
  cli();
  bool e = errL;
  errL = false;
  SREG = oldSREG;
  return e;
}

bool Encoders3pi::checkErrorRight()
{
  begin();
  uint8_t oldSREG = SREG;
  cli();
  bool e = errR;
  errR = false;
  SREG = oldSREG;
  return e;
}

#endif // ENCODERS3PI_IMPLEMENTATION
