// MagnetDetector3pi.h
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <LIS3MDL.h>

// Usage: in exactly one .ino/.cpp file
//   #define MAGNET_DETECTOR3PI_IMPLEMENTATION
//   #include "MagnetDetector3pi.h"
// Other files should only #include it (do not repeat the define)

class MagnetDetector3pi {
public:
  MagnetDetector3pi() = default;

  // 1) Initialization: Wire.begin + mag.init + enableDefault
  bool begin();

  // 2) Calibration: sample "samples" times, with periodMs delay each time (same defaults as original: 50, 20)
  bool calibrate(uint8_t samples = 50, uint16_t periodMs = 20);

  // 3) Detection: read the magnetometer once and compare against the threshold
  //    Returns bool; optionally writes total_diff to strengthOut (so you can print Strength in the .ino)
  bool detect(long* strengthOut = nullptr);

  // 4) Change threshold
  void setThreshold(long threshold) { _threshold = threshold; }

private:
  static inline long labs_long_(long v) { return (v < 0) ? -v : v; }

private:
  LIS3MDL _mag;

  // Save memory: use 1 byte as status flags
  // bit0: inited, bit1: calibrated
  uint8_t _state = 0;

  long _threshold = 9000;//default 3000

  // baseline (same as original: long)
  long _bx = 0, _by = 0, _bz = 0;
};

#ifdef MAGNET_DETECTOR3PI_IMPLEMENTATION

bool MagnetDetector3pi::begin() {
  Wire.begin();

  if (!_mag.init()) {
    _state = 0;
    return false;
  }
  _mag.enableDefault();

  _state = 0x01; // inited
  return true;
}

bool MagnetDetector3pi::calibrate(uint8_t samples, uint16_t periodMs) {
  if (!(_state & 0x01)) return false;
  if (samples == 0) samples = 1;

  long sum_x = 0, sum_y = 0, sum_z = 0;

  for (uint8_t i = 0; i < samples; i++) {
    _mag.read();
    sum_x += (long)_mag.m.x;
    sum_y += (long)_mag.m.y;
    sum_z += (long)_mag.m.z;
    delay(periodMs);
  }

  _bx = sum_x / (long)samples;
  _by = sum_y / (long)samples;
  _bz = sum_z / (long)samples;

  _state |= 0x02; // calibrated
  return true;
}

bool MagnetDetector3pi::detect(long* strengthOut) {
  if (!(_state & 0x01) || !(_state & 0x02)) {
    if (strengthOut) *strengthOut = 0;
    return false;
  }

  _mag.read();

  const long diff_x = labs_long_((long)_mag.m.x - _bx);
  const long diff_y = labs_long_((long)_mag.m.y - _by);
  const long diff_z = labs_long_((long)_mag.m.z - _bz);

  const long total_diff = diff_x + diff_y + diff_z;

  if (strengthOut) *strengthOut = total_diff;
  return total_diff > _threshold;
}

#endif // MAGNET_DETECTOR3PI_IMPLEMENTATION
