#ifndef _LINESENSORS_RC_H
#define _LINESENSORS_RC_H

#include <Arduino.h>

// ===== Pin map (edit if needed) =====
// Common 3pi+: {12, A0, A2, A3, A4}
// Labsheet sometimes uses: {A11, A0, A2, A3, A4}
static const uint8_t LS_PINS[5] = { 12, A0, A2, A3, A4 };
static const uint8_t LS_EMIT_PIN = 11;

class LineSensorsRC
{
public:
  static const uint8_t N = 5;

  // 1) init: initialization + automatically start non-blocking calibration (sweep for calibrate_ms milliseconds)
  inline void init(uint16_t calibrate_ms = 2000)
  {
    // emitters on
    pinMode(LS_EMIT_PIN, OUTPUT);
    digitalWrite(LS_EMIT_PIN, HIGH);

    // sensor pins default
    for (uint8_t i = 0; i < N; i++)
    {
      pinMode(LS_PINS[i], INPUT);
      digitalWrite(LS_PINS[i], LOW);
      _raw[i] = 0;
      _norm[i] = 0;
    }

    _resetMinMax();
    _calibrating = (calibrate_ms > 0);
    _calib_end_ms = _calibrating ? (millis() + calibrate_ms) : 0;
    _calib_frozen = !_calibrating; // if calibrate_ms==0, freeze immediately (degraded range)

    _state = State::Idle;
    _next_start_us = micros(); // start immediately
  }

  // 2) update: non-blocking progression; true = a new frame has just completed
  inline bool update()
  {
    const uint32_t now_us = micros();
    const uint32_t now_ms = millis();

    // Calibration expired: freeze min/max (and apply degraded protection for failed channels)
    if (_calibrating && (int32_t)(now_ms - _calib_end_ms) >= 0)
    {
      _calibrating = false;
      _freezeMinMax();
      _calib_frozen = true;
    }

    switch (_state)
    {
      case State::Idle:
        if ((int32_t)(now_us - _next_start_us) >= 0)
          _startCharge(now_us);
        return false;

      case State::Charging:
        if ((uint16_t)(now_us - _t_charge_start) >= CHARGE_US)
          _startDischarge(now_us);
        return false;

      case State::Discharging:
        return _pollDischarge(now_us);
    }
    return false;
  }

  // 3) readRaw: latest frame raw values (µs)
  inline void readRaw(uint16_t out[N]) const
  {
    for (uint8_t i = 0; i < N; i++) out[i] = _raw[i];
  }

  // 4) readNorm: latest frame normalized values (0..1000)
  inline void readNorm(uint16_t out[N]) const
  {
    for (uint8_t i = 0; i < N; i++) out[i] = _norm[i];
  }

private:
  // ===== fixed timing params (kept minimal; change here if you want) =====
  static const uint16_t TIMEOUT_US = 3000; // max discharge window
  static const uint16_t CHARGE_US  = 10;   // charge duration

  enum class State : uint8_t { Idle, Charging, Discharging };
  State _state = State::Idle;

  uint32_t _t_charge_start = 0;
  uint32_t _t_discharge_start = 0;
  uint32_t _next_start_us = 0;

  uint16_t _raw[N];
  uint16_t _norm[N];

  bool     _done[N];
  uint8_t  _remaining = N;

  // calibration storage (internal)
  uint16_t _min_us[N];
  uint16_t _max_us[N];
  bool     _calibrating = false;
  bool     _calib_frozen = false;
  uint32_t _calib_end_ms = 0;

  inline void _startCharge(uint32_t now_us)
  {
    // charge: set outputs high
    for (uint8_t i = 0; i < N; i++)
    {
      pinMode(LS_PINS[i], OUTPUT);
      digitalWrite(LS_PINS[i], HIGH);
    }
    _t_charge_start = now_us;
    _state = State::Charging;
  }

  inline void _startDischarge(uint32_t now_us)
  {
    for (uint8_t i = 0; i < N; i++)
    {
      pinMode(LS_PINS[i], INPUT);
      digitalWrite(LS_PINS[i], LOW); // no pullup
      _raw[i] = TIMEOUT_US;          // default timeout
      _done[i] = false;
    }
    _remaining = N;
    _t_discharge_start = now_us;
    _state = State::Discharging;
  }

  inline bool _pollDischarge(uint32_t now_us)
  {
    const uint16_t t = (uint16_t)(now_us - _t_discharge_start);

    // poll once per update (non-blocking)
    for (uint8_t i = 0; i < N; i++)
    {
      if (!_done[i] && digitalRead(LS_PINS[i]) == LOW)
      {
        _raw[i] = t;
        _done[i] = true;
        _remaining--;
      }
    }

    // finish if all done or timeout
    if (_remaining == 0 || t >= TIMEOUT_US)
    {
      if (_calibrating && !_calib_frozen)
      {
        for (uint8_t i = 0; i < N; i++)
        {
          if (_raw[i] < _min_us[i]) _min_us[i] = _raw[i];
          if (_raw[i] > _max_us[i]) _max_us[i] = _raw[i];
        }
      }

      _computeNorm1000();

      _next_start_us = now_us; // continuous sampling
      _state = State::Idle;
      return true;
    }

    return false;
  }

  inline void _resetMinMax()
  {
    for (uint8_t i = 0; i < N; i++)
    {
      _min_us[i] = 0xFFFF;
      _max_us[i] = 0;
    }
  }

  inline void _freezeMinMax()
  {
    // Apply degraded protection for invalid channels to ensure the normalization range is valid
    for (uint8_t i = 0; i < N; i++)
    {
      if (_max_us[i] <= _min_us[i])
      {
        _min_us[i] = 0;
        _max_us[i] = TIMEOUT_US;
      }
    }
  }

  inline void _computeNorm1000()
  {
    for (uint8_t i = 0; i < N; i++)
    {
      uint16_t mn = 0;
      uint16_t mx = TIMEOUT_US;

      // During/after calibration: use the range as long as it's valid; otherwise degrade
      if ((_calibrating || _calib_frozen) && (_max_us[i] > _min_us[i]))
      {
        mn = _min_us[i];
        mx = _max_us[i];
      }

      uint16_t range = (mx > mn) ? (uint16_t)(mx - mn) : 1;

      int32_t x = (int32_t)_raw[i] - (int32_t)mn;
      if (x < 0) x = 0;
      if (x > (int32_t)range) x = range;

      _norm[i] = (uint16_t)((x * 1000L) / range); // 0..1000
    }
  }
};

#endif
