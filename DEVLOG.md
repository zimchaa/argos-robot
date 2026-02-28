# ARGOS Dev Log

## Session 1 — Phase 1: Core drivers

### What was done
- Established project structure: `argos/drivers/`, `argos/base/`, `argos/arm/`
- Read both vendor libraries before writing any code:
  - Waveshare Motor Driver HAT (`PCA9685.py` + `MotorDriver.py`)
  - SB Components MotorShield (GPIO-based, not I2C)
- Wrote `argos/drivers/pca9685.py` — unified PCA9685 driver using smbus2 only
  - `PCA9685`: low-level register access (set_pwm, set_duty, set_level, set_frequency)
  - `I2CMotor`: single-motor abstraction with `run(speed)` interface; motors on the same chip share a PCA9685 instance via module-level cache
- Wrote `argos/drivers/gpio_motor.py` — GPIO motor driver using RPi.GPIO
  - `GPIOMotor`: single-motor abstraction with identical `run(speed)` interface
  - Direction config uses neutral `pin_a`/`pin_b` names; swap in `_MOTOR_PINS` to reverse a motor without code changes
- Wrote `argos/base/tracks.py` — `TrackedBase` with `forward`, `backward`, `turn_left`, `turn_right`, `pivot_left`, `pivot_right`, `stop`
- Wrote `argos/arm/joints.py` — `RobotArm` with named joint attributes (`shoulder`, `elbow`, `wrist`, `gripper`), each a `GPIOMotor`

### Key design decisions
- Both `I2CMotor` and `GPIOMotor` expose identical `run(speed)` interface; speed is -100..100 signed int, no direction strings at driver level
- Waveshare HAT (I2C 0x40): motors 0 and 1 — tracks
- SB Components MotorShield (GPIO BOARD pins): motors 1–4 — arm joints
- SPI pins reused as GPIO on MotorShield (motors 3 and 4); safe with SPI disabled

### What worked
- Project structure and driver design complete and consistent
- No hardware testing yet (development platform: Pi 400, target: Pi 4 @ argos.local)

### Unresolved
- Hardware not yet connected; all code untested on physical hardware
- HAT 2 (SB Components) I2C address confirmed as N/A — it is GPIO-based
- GPIO pin conflict check between stacked HATs still TBD on hardware
- No i2cdetect run yet to confirm Waveshare HAT appears at 0x40
- `smbus2` and `RPi.GPIO` not yet installed on target Pi 4

---

## Session 2 — Hardware bring-up: tracks confirmed

### What was done
- Ran `i2cdetect -y 1` — Waveshare HAT confirmed at 0x40 (all-call at 0x70 also present, normal)
- Confirmed `smbus2` and `RPi.GPIO` installed on target Pi 4
- Ran `tests/test_tracks_manual.py` — both tracks spin and run in the correct direction at 70%
- Extended test script (renamed to `tests/test_all_motors_manual.py`) to cover arm joints (shoulder/elbow/wrist/gripper) at 25% / 0.5 s — no governor fitted yet

### What worked
- Waveshare Motor Driver HAT I2C comms verified (MODE1 = 0x11, oscillator running)
- Left track (Motor 0) and right track (Motor 1) both respond correctly, correct direction for positive speed
- `TrackedBase.forward()` drives both tracks together as expected

### Unresolved
- Arm joint directions not yet verified on hardware — run the extended test and note which direction each joint moves for +ve speed; swap `pin_a`/`pin_b` in `gpio_motor._MOTOR_PINS` for any that are reversed
- GPIO pin conflicts between stacked HATs not yet confirmed (tracks working suggests no conflict on I2C pins; GPIO pins still TBD)
- No governor / joint angle limits in place — arm testing must be done carefully at low speed

---

## Session 5 — Vision + sensorium hardware bring-up (2026-02-27)

### What was done
- Plugged in USB webcam; wrote `argos/vision/camera.py` — `Camera` class wrapping `cv2.VideoCapture`
- Wrote `tests/test_camera.py` — headless test: 5-frame warm-up, saves `tests/camera_test.jpg`
- Confirmed all sensorium sensors connected and active on hardware:
  - **MPU-6050** IMU — I2C 0x68 on Waveshare expansion header (gyro + accel)
  - **Flotilla Motion ×2** — LSM303D (accel + magnetometer), via Flotilla dock USB
  - **Flotilla Weather** — BMP280 (temperature + pressure), via Flotilla dock USB
  - **IR proximity ×2** — BOARD 7 (CN9) and BOARD 12 (CN8)
  - **Camera** — USB webcam at /dev/video0

### Camera test result
```
Resolution : 640 × 480
FPS        : 30.0
Frame shape: (480, 640, 3)  dtype=uint8
```

### What worked
- Camera opens immediately, warm-up and frame save work correctly
- All sensorium sensor hardware confirmed present and responsive
- `argos/vision/` package structure established

### What else was done this session
- Wrote `argos/sensorium/imu.py` — `MPU6050` + `ImuReading` (14-byte I2C burst, accel/gyro/temp properties)
- Wrote `argos/sensorium/sonar.py` — `HCSR04` (10 μs trigger, echo timing, temperature-corrected speed of sound)
- Wrote `argos/sensorium/ir.py` — `IRSensor` + `IRPair` (digital, active-low, pull-up, BOARD 7/12)
- Wrote `argos/sensorium/flotilla.py` — `FlotillaReader` (USB serial, background thread, Motion ×2, Weather, Colour)
  - Fixed two bugs in Pimoroni's official heading formula (normalisation + tiltcomp_y sign error)
- Wrote `tests/test_imu.py`, `tests/test_ir.py`, `tests/test_flotilla.py`, `tests/test_sonar.py`
- HC-SR04 sonar confirmed working: readings from 3.4 cm to 23.2 cm verified against known distances

### Remaining after this session
- AHRS filter not yet written: `ahrs.py` (Madgwick — MPU-6050 gyro/accel + LSM303D mag → roll/pitch/yaw)
- `fusion.py` + `target.py` not yet written (Sensorium integration layer)
- Camera calibration (intrinsics) not yet done
- ArUco detection module (`argos/vision/aruco.py`) not yet written

---

## Session 3 — Full motor test: all 6 motors verified

### What was done
- Ran `tests/test_all_motors_manual.py` with interactive reporting at 70% speed / 1s
- Confirmed all track and arm motor drivers communicate and respond correctly
- Discovered arm joint wiring did not match assumed motor ID order; corrected in `joints.py`

### What worked
- Both tracks spin forward with positive speed; correct direction confirmed
- Elbow (motor 2) and wrist (motor 3) correctly connected and correct polarity
- Gripper (physically on motor 1 terminal) responds and opens on positive speed
- Arm joint motor driver (GPIOMotor + MotorShield) confirmed functional

### Findings and fixes
- **Arm wiring swap**: motor 1 terminal → gripper, motor 4 terminal → shoulder
  (was assumed to be shoulder=1, gripper=4). Fixed in `argos/arm/joints.py` — no rewiring needed.
- **Track drift**: motor 1 (right track) appears mechanically slower than motor 0 at equal power.
  Noted for future compensation — will need feedback (IMU/encoder/visual) to correct automatically.
- **Shoulder (motor 4) no movement**: motor 4 terminal shows no movement. Possible causes:
  loose connector, mechanical bind, or faulty motor. Needs physical investigation before arm use.

### Unresolved
- Shoulder joint not moving — investigate connector and motor 4 terminal wiring
- Track drift to be quantified and compensated once odometry/feedback is available
- Joint directions (positive = which physical way) not yet fully characterised for all joints

---

## Session 4 — AHRS + sensor axis calibration (2026-02-27, continued)

### What was done
- Wrote `argos/sensorium/ahrs.py` — `MadgwickAHRS` (9DOF MARG + 6DOF IMU-only modes)
  - Gradient-descent quaternion update; Euler output (roll/pitch/yaw in degrees)
  - `init_from_accel()` for fast initial convergence; `calibrate_level()` for mounting offset
  - Uses body Motion sensor (ch 6) as the magnetometer source
- Added sensorium constants to `config.py`: `IMU_I2C_BUS/ADDR`, `FLOTILLA_BODY_MOTION_CH`,
  `FLOTILLA_ARM_MOTION_CH`, `SONAR_TRIG/ECHO_PIN`, `IR_PIN_1/2`
- Wrote `tests/probe_sensor_axes.py` — live display of all three IMU chips simultaneously
  to enable physical tilt tests for axis-to-filter-frame mapping
- Ran three-position probe:
  - **Flat** (gravity = −Z): MPU chip+X=down; Body LSM chip+X=down; Arm LSM chip+X=up
  - **Nose-down** (gravity = +X): MPU chip+Z=fwd; Body LSM chip+Y=fwd; Arm LSM chip−Y=fwd
  - **Right-tilt 90°** (gravity = +Y): MPU chip+Y=right ✓ (RHR confirmed);
    Body LSM chip+Z=right; Arm LSM chip+Z=right (both LSMs left-handed convention — RHR wrong)
- Wrote full `IMU_AXIS_REMAP`, `BODY_MOTION_AXIS_REMAP`, `ARM_MOTION_AXIS_REMAP` into
  `config.py` — all axes ★ measured, no inferred values

### Key findings
- Both Flotilla LSM303D chips use a **left-handed** axis convention: Z = −(X × Y).
  The right-hand rule predicts `chip+Z = LEFT` from the other two axes, but hardware
  shows `chip+Z = RIGHT`. The corrected filter-frame mapping is `filter_Y = +chip_z`
  (was `−chip_z` before the right-tilt test).
- MPU-6050 is standard right-handed — right-hand rule holds for all three axes.

---

## Session 6 — Magnetometer compass spin test + calibration (2026-02-28)

### What was done
- Ran compass spin test: robot flat on table, rotated CW in 90° steps (0°, 90°, 180°)
- Analysed spin test readings against BODY_MOTION_MAG_REMAP:
  - `heading = atan2(−mz_cal, my_cal)` increases clockwise ✓
  - `mx` (vertical field component via `−chip_x`) varies < 15% across yaw — stable ✓
  - **BODY_MOTION_MAG_REMAP confirmed correct** — no config change needed
- Quantified hard-iron bias:
  - `mz` offset ≈ +9938 (≈ 5× signal amplitude — motor permanent magnets nearby)
  - `my` offset ≈ +3127; `mx` offset negligible (vertical component)
  - From half-rotation: `my_bias = (my_0 + my_180) / 2 = 3127`, `mz_bias = (mz_0 + mz_180) / 2 = 9938`
- Added `MAG_HARD_IRON_BIAS = (0, 3127, 9938)` to `config.py`, flagged as rough / half-rotation only
- Full 360° spin still needed to finalise bias values

### What worked
- Remap confirmed with quantitative circle fit (radius ≈ 1866 counts, consistent across all 3 points)
- 0° and 180° readings give `(my − bias)` and `(mz − bias)` equal and opposite to within ±2% ✓

### Remaining after this session
- Full 360° horizontal spin to refine `MAG_HARD_IRON_BIAS` (min/max method)
- `argos/sensorium/fusion.py` — Sensorium integration class (Phase 2e)
- `argos/sensorium/target.py` + `argos/sensorium/floor_plane.py` (Phase 2e)
- `argos/vision/aruco.py` — ArUco detection (Phase 2a)
- `argos/vision/calibration/` — camera intrinsics (Phase 2a)
- `argos/arm/kinematics.py` — FK + IK solver (Phase 2b)
- `argos/planner/` — goal decomposition + executor (Phase 2c)
- `argos/mcp/` — MCP server (Phase 3)
