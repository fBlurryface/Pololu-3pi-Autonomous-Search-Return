#pragma once
#include <Arduino.h>

// Minimal motor driver layer: only outputs signed PWM commands to the pins
// Speed range: -255 ~ +255 (positive = forward, negative = reverse)
class Motors3pi {
public:
  // forwardLevel: logic level output on the DIR pin when moving forward (LOW/HIGH); can be set separately for left/right
  inline Motors3pi(bool leftForwardLevel = LOW, bool rightForwardLevel = LOW)
  : leftFwdLevel_(leftForwardLevel), rightFwdLevel_(rightForwardLevel) {}

  inline void begin() {
    pinMode(PIN_LEFT_DIR,  OUTPUT);
    pinMode(PIN_RIGHT_DIR, OUTPUT);
    pinMode(PIN_LEFT_PWM,  OUTPUT);
    pinMode(PIN_RIGHT_PWM, OUTPUT);
    coast();
  }

  inline void set(int16_t leftCmd, int16_t rightCmd) {
    driveOne_(PIN_LEFT_DIR,  PIN_LEFT_PWM,  leftCmd,  leftFwdLevel_);
    driveOne_(PIN_RIGHT_DIR, PIN_RIGHT_PWM, rightCmd, rightFwdLevel_);
  }

  inline void setLeft(int16_t leftCmd) {
    driveOne_(PIN_LEFT_DIR, PIN_LEFT_PWM, leftCmd, leftFwdLevel_);
  }

  inline void setRight(int16_t rightCmd) {
    driveOne_(PIN_RIGHT_DIR, PIN_RIGHT_PWM, rightCmd, rightFwdLevel_);
  }

  inline void coast() {
    analogWrite(PIN_LEFT_PWM,  0);
    analogWrite(PIN_RIGHT_PWM, 0);
  }

  inline void setForwardLevels(bool leftForwardLevel, bool rightForwardLevel) {
    leftFwdLevel_  = leftForwardLevel;
    rightFwdLevel_ = rightForwardLevel;
  }

private:
  // Fixed pins on 3pi+ 32U4
  static constexpr uint8_t PIN_RIGHT_DIR = 15;
  static constexpr uint8_t PIN_LEFT_DIR  = 16;
  static constexpr uint8_t PIN_RIGHT_PWM = 9;
  static constexpr uint8_t PIN_LEFT_PWM  = 10;

  bool leftFwdLevel_;
  bool rightFwdLevel_;

  static inline int16_t clamp_(int16_t x) {
    if (x > 255) return 255;
    if (x < -255) return -255;
    return x;
  }

  static inline void driveOne_(uint8_t pinDir, uint8_t pinPwm, int16_t cmd, bool forwardLevel) {
    cmd = clamp_(cmd);
    bool forward = (cmd >= 0);
    uint8_t pwm = (uint8_t)abs(cmd);

    digitalWrite(pinDir, forward ? forwardLevel : !forwardLevel);
    analogWrite(pinPwm, pwm);
  }
};
