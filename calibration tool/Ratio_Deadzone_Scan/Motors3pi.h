#pragma once
#include <Arduino.h>

// 极简电机驱动层：只负责把 signed PWM 命令输出到引脚
// 速度范围：-255 ~ +255（正数=前进，负数=后退）
class Motors3pi
{
public:
  // forwardLevel: 正数=前进时 DIR 脚输出的电平（LOW/HIGH），左右可分别设置
  inline Motors3pi(bool leftForwardLevel = LOW, bool rightForwardLevel = LOW)
  : leftFwdLevel_(leftForwardLevel), rightFwdLevel_(rightForwardLevel) {}

  inline void begin()
  {
    pinMode(PIN_LEFT_DIR,  OUTPUT);
    pinMode(PIN_RIGHT_DIR, OUTPUT);
    pinMode(PIN_LEFT_PWM,  OUTPUT);
    pinMode(PIN_RIGHT_PWM, OUTPUT);
    coast();
  }

  inline void set(int16_t leftCmd, int16_t rightCmd)
  {
    driveOne_(PIN_LEFT_DIR,  PIN_LEFT_PWM,  leftCmd,  leftFwdLevel_);
    driveOne_(PIN_RIGHT_DIR, PIN_RIGHT_PWM, rightCmd, rightFwdLevel_);
  }

  inline void setLeft(int16_t leftCmd)
  {
    driveOne_(PIN_LEFT_DIR, PIN_LEFT_PWM, leftCmd, leftFwdLevel_);
  }

  inline void setRight(int16_t rightCmd)
  {
    driveOne_(PIN_RIGHT_DIR, PIN_RIGHT_PWM, rightCmd, rightFwdLevel_);
  }

  inline void coast()
  {
    analogWrite(PIN_LEFT_PWM,  0);
    analogWrite(PIN_RIGHT_PWM, 0);
  }

  inline void setForwardLevels(bool leftForwardLevel, bool rightForwardLevel)
  {
    leftFwdLevel_  = leftForwardLevel;
    rightFwdLevel_ = rightForwardLevel;
  }

private:
  // 3pi+ 32U4 固定引脚
  static constexpr uint8_t PIN_RIGHT_DIR = 15;
  static constexpr uint8_t PIN_LEFT_DIR  = 16;
  static constexpr uint8_t PIN_RIGHT_PWM = 9;
  static constexpr uint8_t PIN_LEFT_PWM  = 10;

  bool leftFwdLevel_;
  bool rightFwdLevel_;

  static inline int16_t clamp_(int16_t x)
  {
    if (x > 255) return 255;
    if (x < -255) return -255;
    return x;
  }

  static inline void driveOne_(uint8_t pinDir, uint8_t pinPwm,
                               int16_t cmd, bool forwardLevel)
  {
    cmd = clamp_(cmd);
    bool forward = (cmd >= 0);
    uint8_t pwm = (uint8_t)abs(cmd);

    digitalWrite(pinDir, forward ? forwardLevel : !forwardLevel);
    analogWrite(pinPwm, pwm);
  }
};
