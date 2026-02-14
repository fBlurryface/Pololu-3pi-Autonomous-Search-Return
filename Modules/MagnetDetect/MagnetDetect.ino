#define MAGNET_DETECTOR3PI_IMPLEMENTATION
#include "MagnetDetector3pi.h"

// --- Hardware configuration ---
const int BUZZER_PIN = 6;

// --- Threshold setting ---
const int MAGNET_THRESHOLD = 3000;

MagnetDetector3pi magnet;

void setup() {
  Serial.begin(9600);

  pinMode(BUZZER_PIN, OUTPUT);

  if (!magnet.begin()) {
    while (1) {
      Serial.println("Failed to detect magnetometer!");
      delay(1000);
    }
  }

  magnet.setThreshold(MAGNET_THRESHOLD);

  // --- Calibration phase (same order as the original version) ---
  Serial.println("Calibrating... Keep still!");

  tone(BUZZER_PIN, 1000, 500);

  magnet.calibrate(50, 20);

  Serial.println("Calibration Done. Ready!");
  tone(BUZZER_PIN, 2000, 100); delay(150);
  tone(BUZZER_PIN, 2000, 100);
}

void loop() {
  long total_diff = 0;
  bool detected = magnet.detect(&total_diff);

  if (detected) {
    Serial.print("MAGNET DETECTED! Strength: ");
    Serial.println(total_diff);

    tone(BUZZER_PIN, 2000, 50);
    delay(100);
  } else {
    noTone(BUZZER_PIN);

    Serial.print("Scanning... Strength: ");
    Serial.println(total_diff);

    delay(100);
  }
}

