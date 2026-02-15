# Pololu-3pi-Autonomous-Search-Return
Control code for Pololu 3pi+ 32U4 robot.

1. **Start initialization** (sensor calibration + establish a reference heading)
2. **Coverage-style search** for a magnet inside the mapped area  
   (line-edge avoidance + a “virtual boundary” to keep the robot inside the region)
3. **Return-to-home trigger**: **magnet found** OR **time limit reached**
4. **Return home** (stop → turn toward origin → drive back → hold)

Designed for **power-on auto-run**: flash the main firmware, power the robot, it starts the mission.

---

## Task map

GitHub does **not** reliably render PDFs inline inside README.  
So the README shows a PNG preview, and clicking the image opens the PDF.

[![Task Map](figure/Map_A2_preview.png)](figure/Map_A2.pdf)

- PDF source: `figure/Map_A2.pdf`

---

## Demo video (playable window on the repo page)

### Option A (best on GitHub): inline MP4 player in README

GitHub blocks `<iframe>`, so you can’t embed a YouTube player directly.  
But GitHub **can** show an inline video player if you attach an MP4 to the README:

1. On GitHub, edit `README.md`
2. Drag-and-drop `demo.mp4` into the editor
3. GitHub inserts a `https://github.com/user-attachments/assets/...` URL
4. Keep that URL **alone on its own line** → README will display a playable video window

Paste the generated link here (keep it as a single line):

<!-- https://github.com/user-attachments/assets/XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX -->

### Option B: YouTube thumbnail link (not inline playback)

Replace `VIDEO_ID` with your YouTube video ID:

[![YouTube demo](https://youtube.com/shorts/DVe-mRGxPlA)

---

## Motor deadzone figure (placeholder slot)

Replace this with your final deadzone visualization (PWM→CPS curve, moving probability, etc.).

![Deadzone Placeholder](figure/deadzone_placeholder.png)

---

## Repository structure

- `Main_Firmware/`
  - Main mission firmware (FSM + motion shaping + odometry + sensor triggers)
- `Modules/`
  - Standalone “API showcase” sketches for individual modules (LineSensors / Magnet / Motion / Odometry)
- `calibration tool/`
  - Calibration & measurement sketches (straight counts, wheelbase turn, backlash measurement, deadzone scans, etc.)
- `figure/`
  - Map + README figures

---

## Dependencies

### Hardware
- Pololu **3pi+ 32U4**

### Arduino libraries
- **Pololu LIS3MDL** (magnetometer) — provides `LIS3MDL.h`

Everything else (encoders, motor driver layer, RC line sensors, odometry, speed control) is implemented in this repo.

---

## Quick start

1. Open `Main_Firmware/Main_Firmware.ino` in Arduino IDE
2. Select a board compatible with **ATmega32U4**
3. Install the **Pololu LIS3MDL** library
4. Build & upload

On power-up, the robot will:
- calibrate and establish a reference heading (“east” reference)
- drive forward to set the virtual boundary origin
- start exploration (edge avoidance + virtual boundary)
- continuously check the magnetometer
- return home when triggered (magnet or timeout)

---

## How it works (module relationships)

### Sensors
- **RC line sensors** — `LineSensorsRC.h`  
  Non-blocking RC charge/discharge timing, auto-calibration (min/max), normalized 0..1000.  
  Used for **edge detection** and avoidance.
- **Magnetometer** — `MagnetDetector3pi.h`  
  Baseline calibration + threshold detection during exploration.

### Motion
- **Encoders** — `Encoders3pi.h`  
  Interrupt-based counts + windowed speed estimate (counts/s).
- **Wheel speed control** — `WheelSpeedCtrl3pi.h`  
  Sets target wheel speed (CPS) → outputs motor command (PID + clamps).

### Odometry / Navigation
- **Differential-drive odometry** — `OdomDiff2W_Basic.h`  
  Integrates `(x, y, th)` from encoder deltas.
- **Return-home logic**  
  stop → turn-to-origin → drive straight back → hold.

---

## Common tuning knobs

Most knobs are at the top of the main firmware:
- time limit / mission timeout
- line thresholds (edge/white)
- magnet detection threshold
- cruise & pivot speeds (CPS)
- settle times / lockouts
- motion shaping limits (accel limiters, PID clamps)

---

## Notes

- Odometry is encoder-based: wheel slip will accumulate error.
- The “map boundary” is enforced via **line edge avoidance** + **virtual boundary logic**, not a full localization stack.

---

## License

Add your chosen license (MIT / Apache-2.0 / GPL-3.0 / etc.).
