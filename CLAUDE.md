# ARGOS Robot — Claude Context

## Project overview

ARGOS is a Python robotics control library for a Raspberry Pi 4 robot
(`argos.local`). Tracked chassis + 4-joint OWI-535 arm. End goal: expose the
robot to a remote multimodal LLM via MCP, with onboard vision (ArUco markers),
kinematics, and a safety layer that has unconditional authority over all motors.

Development platform: Raspberry Pi 400.

---

## Repository layout

```
argos/
  config.py           # Central hardware config — MotorConfig dataclass, TRACK_MOTORS, ARM_JOINTS
  drivers/
    pca9685.py        # PCA9685 I2C driver + I2CMotor (tracks, via Waveshare HAT)
    gpio_motor.py     # GPIOMotor (arm joints, via SB Components MotorShield)
  base/
    tracks.py         # TrackedBase — differential drive controller
  arm/
    joints.py         # RobotArm — 4 named joint attributes (shoulder/elbow/wrist/gripper)
    kinematics.py     # [planned] FK + IK (see docs/owi535_ik_reference.md)
  safety/
    monitor.py        # SafetyMonitor — speed clamping + watchdog (LIVE)
  vision/
    camera.py         # USB webcam capture (OpenCV VideoCapture) — LIVE
    aruco.py          # [planned] ArUco detection, joint angle extraction
    calibration/      # [planned] Camera intrinsics + distortion data
  sensorium/
    fusion.py         # [planned] Sensorium — fused state from all sensors
    floor_plane.py    # [planned] Monocular XZ estimation via ground-plane ray-cast
    imu.py            # MPU-6050 driver (I2C 0x68) — LIVE
    flotilla.py       # Flotilla dock wrapper — Motion ×2, Weather, Colour — LIVE
    ahrs.py           # [planned] Madgwick filter — MPU-6050 + LSM303D → roll/pitch/yaw
    sonar.py          # HC-SR04 driver (BOARD 29/31, divider fitted) — LIVE
    ir.py             # IR proximity drivers (BOARD 7/12) — LIVE
    target.py         # [planned] TargetEstimate dataclass + confidence model
  planner/
    goal.py           # [planned] Goal dataclass (target xyz, grip, tolerance)
    decompose.py      # [planned] Decompose 3D goal → base moves + arm angles
    executor.py       # [planned] Execute plan with ArUco + Sensorium feedback
  mcp/
    server.py         # [planned] MCP tool definitions
    __main__.py       # [planned] python -m argos.mcp entry point
tests/
  test_all_motors_manual.py   # Interactive hardware test — all motors
  motor_jog.py                # Single-motor jog via SafetyMonitor
  jig.py                      # Random multi-motor sequence
requirements.txt    # smbus2, RPi.GPIO (+ opencv-python, mcp planned)
DEVLOG.md           # Session-by-session development log
docs/
  roadmap.md                            # Forward plan and procurement list
  camera_mount.md                       # Camera boom design and FOV calculations
  owi535_ik_reference.md               # IK algorithm reference (Debenec 2015)
  robot_side.jpeg / robot_top.jpeg     # Hardware photos
  waveshare_motor_driver_hat_schematic.pdf
  sbcomponents_motorshield_schematic_v1.3.pdf
```

---

## Architecture

Three layers (bottom to top):

**1. Drivers** — `argos/drivers/`
- `I2CMotor` — single DC motor on Waveshare HAT via shared `PCA9685` instance.
- `GPIOMotor` — single DC motor on SB Components MotorShield via RPi.GPIO PWM.
- Both expose `run(speed)` where speed is **-100 to 100**.

**2. Controllers** — `argos/base/`, `argos/arm/`
- `TrackedBase` — wraps two `I2CMotor` instances with named drive commands.
- `RobotArm` — wraps four `GPIOMotor` instances as named joint attributes.

**3. Safety** — `argos/safety/monitor.py` (LIVE)
- `SafetyMonitor` wraps both controllers. All production code must go through it.
- Speed clamping per motor: `max_speed` and `min_speed` from `config.py`.
- Watchdog: each `run()` call registers a deadline; motor auto-stops at `max_duration`.
- `emergency_stop()` kills every motor unconditionally.
- Exposes `safety.base` (`SafeBase`) and `safety.arm` (`SafeArm`) with the same
  interface as `TrackedBase`/`RobotArm`.

---

## Hardware mapping

### Tracks — Waveshare Motor Driver HAT (I2C 0x40, bus 1)

| Motor ID | Track  | PCA9685 channels              |
|----------|--------|-------------------------------|
| 0        | Left   | PWM=ch0, IN1=ch1, IN2=ch2    |
| 1        | Right  | PWM=ch5, IN1=ch3, IN2=ch4    |

Direction: positive speed → IN1=0, IN2=1 · negative → IN1=1, IN2=0.

### Arm — SB Components MotorShield (GPIO BOARD pins)

**Wiring confirmed on hardware 2026-02-24.**

| Motor ID | Joint    | enable | pin_a | pin_b |
|----------|----------|--------|-------|-------|
| 1        | gripper  | 11     | 15    | 13    |
| 2        | elbow    | 22     | 16    | 18    |
| 3        | wrist    | 19     | 21    | 23    |
| 4        | shoulder | 32     | 24    | 26    |

Motors 3 and 4 use SPI pins — safe with SPI disabled (default on Pi OS Lite).
To reverse a joint's effective direction, swap `pin_a`/`pin_b` in `gpio_motor._MOTOR_PINS`.

### MotorConfig limits (from `argos/config.py`)

| Joint    | max_speed | min_speed | max_duration |
|----------|-----------|-----------|--------------|
| left     | 100 %     | 30 %      | 10 s         |
| right    | 100 %     | 30 %      | 10 s         |
| shoulder | 80 %      | 25 %      | 3 s          |
| elbow    | 80 %      | 25 %      | 3 s          |
| wrist    | 80 %      | 25 %      | 3 s          |
| gripper  | 80 %      | 20 %      | 2 s          |

---

## Key conventions

- **Speed range**: -100..100. Both drivers clamp. Safety layer also applies per-motor limits.
- **GPIO mode**: always `GPIO.BOARD`. Never mix BCM. `RobotArm.__init__` sets it.
- **PCA9685 sharing**: `_pca_instances` dict keyed by `(bus, address)` — one connection per chip.
- **Entry point**: use `SafetyMonitor`, not raw controllers, for any motor motion.
- **Shutdown**: `safety.close()` stops everything and releases hardware.
- **Config**: motor IDs, directions, limits all live in `argos/config.py` — nowhere else.
- **No vendor libraries**: drivers written from scratch using `smbus2` + `RPi.GPIO` only.

---

## Recommended usage

```python
from argos.safety.monitor import SafetyMonitor

safety = SafetyMonitor()
try:
    safety.base.forward(60)           # clamped, auto-stops after 10 s
    safety.arm.shoulder.run(40)       # clamped to 80 %, auto-stops after 3 s
    safety.emergency_stop()           # kills everything immediately
finally:
    safety.close()
```

---

## Hardware details

### Waveshare Motor Driver HAT
- PCA9685 channels 6–15 are free (unconnected on board) — usable for servos over I2C.
- I2C expansion header (5-pin): VIN, 3V3, GND, SDA, SCL — ideal for IMU (MPU-6050 at 0x68).
- Address solder pads R8–R12: populate to shift address from 0x40 up to 0x7F.

### SB Components MotorShield free pins (BOARD numbering)

| BOARD | BCM    | Available for             |
|-------|--------|---------------------------|
| 29    | GPIO5  | Ultrasonic TRIG (CN10)    |
| 31    | GPIO6  | Ultrasonic ECHO (CN10)    |
| 7     | GPIO4  | IR sensor 1 (CN9)         |
| 12    | GPIO18 | IR sensor 2 (CN8)         |
| 33    | GPIO13 | Direction LED group T2    |
| 35    | GPIO19 | Direction LED group T1    |
| 36    | GPIO16 | Direction LED group T3    |
| 37    | GPIO26 | Direction LED group T4    |
| **38**| GPIO20 | **FREE**                  |
| **40**| GPIO21 | **FREE**                  |

CN10 has a 1K/2K voltage divider fitted — HC-SR04 ECHO is already level-shifted.
BOARD 33/35/36/37 drive onboard direction LEDs — **not free for other use**.
Use BOARD 38/40 + GND at BOARD 39 for any external LED or signal.

---

## Vision subsystem (planned)

- **Camera**: USB webcam (decided). OpenCV `VideoCapture`. 640×480 or 720p @ 30 fps.
  Pi Camera Module CSI port reserved for a possible future forward-facing nav camera.
- **Mount**: side-mounted boom, ~30–40 cm lateral offset, pointing at arm face-on.
  See `docs/camera_mount.md` for design options and FOV calculations.
- **Pose tracking**: ArUco markers (`DICT_4X4_50`) on each arm link.
  One marker per link → 6-DOF pose → adjacent poses → joint angles.
- **Calibration**: intrinsic matrix + distortion stored in `argos/vision/calibration/`.
- **LLM frame delivery**: `get_camera_frame()` MCP tool → base64 JPEG.

---

## Arm kinematics (planned)

Vertical-plane serial chain (no base rotation — base is fixed).
Full IK algorithm: see `docs/owi535_ik_reference.md` (Debenec 2015).

| Segment | Length  |
|---------|---------|
| Shoulder pivot height above deck | ~9 cm |
| L2: shoulder → elbow   | 9.0 cm  |
| L3: elbow → wrist      | 11.3 cm |
| L4: wrist → gripper tip | 10.5 cm |

Joint limits for safety layer (use measured values, not spec):

| Joint    | Measured range | Safety limit |
|----------|----------------|--------------|
| Shoulder | 194°           | 180°         |
| Elbow    | 260°           | 260°         |
| Wrist    | 100°           | 100°         |
| Gripper  | 0–4.5 cm       | —            |

---

## Sensorium (planned)

Owns all environment-facing sensors and exposes a single fused `TargetEstimate`
to the planner. Implements a coarse-to-fine target acquisition model — the planner
does not need precise coordinates upfront, just enough to start moving.

```
Distance    Active sensors                    Confidence
────────    ──────────────────────────────    ──────────
> 1 m       Camera bearing + AHRS heading     LOW — approach
            + floor-plane if on ground
20–100 cm   Camera + sonar + AHRS             MEDIUM — refine
5–30 cm     Camera + sonar + IR + Colour      HIGH — plan
            + ArUco on target (if any)        HIGHEST
```

Key sensors:
- **AHRS** (Madgwick filter): MPU-6050 gyro + accel + Flotilla LSM303D magnetometer → stable roll/pitch/yaw with absolute compass heading. Corrects gyro drift on turns. Active at all ranges.
- **Floor-plane homography**: AHRS roll/pitch corrects camera tilt for ground-plane ray-cast. Gives XZ position for floor targets without stereo.
- **Sonar + camera bearing**: polar → cartesian for non-floor targets.
- **Colour module** (Flotilla): RGB + ambient light at close range. Object colour ID for task context. Ambient light predicts ArUco detection reliability.
- **Arm Motion module** (Flotilla, second LSM303D on shoulder link): accelerometer tilt → shoulder joint angle when ArUco is occluded.
- Neural depth (MiDaS etc.) explicitly excluded — too slow on Pi 4 CPU.

See `docs/roadmap.md` — Phase 2e for full design, module structure, Pi 4 thread
budget, calibration requirements, and Flotilla integration details.

---

## Planner layer (planned)

Sits between the MCP server and the safety layer. Decomposes a 3D goal into base
movements + arm joint targets, then executes them with feedback loops. Runs entirely
on the Pi — no LLM inference required during execution.

```
move_arm(x, y, z, grip)
    │
    ├─ 1. Turn base to face target         ← IMU gyro feedback (MPU-6050)
    ├─ 2. Drive to correct standoff        ← time-based dead reckoning
    ├─ 3. Solve 2D IK → joint angles       ← kinematics.py (Debenec algorithm)
    ├─ 4. Servo arm to target angles       ← ArUco visual feedback (closed loop)
    └─ 5. Actuate gripper
```

The arm's visual servo loop corrects residual base positioning error.
The time-based safety watchdog remains as a backstop if ArUco is occluded.

See `docs/roadmap.md` — Phase 2c for full algorithm, coordinate system,
fallback behaviour, and calibration parameters.

---

## MCP interface (planned)

| Tool                | Args              | Returns           | Notes                        |
|---------------------|-------------------|-------------------|------------------------------|
| `get_camera_frame`  | —                 | base64 JPEG       | LLM visual context           |
| `get_arm_pose`      | —                 | angles + xyz      | Current state                |
| `move_arm`          | `x, y, z, grip`   | success/error     | IK → motor + visual feedback |
| `move_base_forward` | `cm: float`       | success/error     | Dead-reckoning                |
| `turn_base`         | `degrees: float`  | success/error     | +ve = right                  |
| `stop_all`          | —                 | —                 | Emergency stop                |

Transport: SSE (network preferred). Library: Anthropic `mcp` package.
Entry point: `python -m argos.mcp`.

---

## Development status

Session 5 complete (2026-02-27). All hardware verified. All sensorium sensors connected.

**Confirmed working:**
- Waveshare HAT at I2C 0x40 (`i2cdetect -y 1` verified)
- Both tracks forward on positive speed; `TrackedBase.forward()` drives together
- All four arm joints confirmed working (2026-02-26): shoulder, elbow, wrist, gripper
- `SafetyMonitor` speed clamping and watchdog verified via `motor_jog.py`
- **USB webcam** at /dev/video0 — 640×480 @ 30fps confirmed (`tests/test_camera.py`)
- **MPU-6050 IMU** — connected at I2C 0x68 on Waveshare expansion header
- **Flotilla Motion ×2** (LSM303D accel + magnetometer) — connected via Flotilla dock USB
- **Flotilla Weather** (BMP280 temperature + pressure) — connected via Flotilla dock USB
- **IR proximity ×2** — BOARD 7 (CN9) and BOARD 12 (CN8)

**Known issues:**
- **Track drift**: right track (motor 1) runs slower than left at equal power.
  Needs IMU/encoder/visual feedback to compensate.

- **HC-SR04 sonar** — confirmed working (readings 3–23 cm verified, `tests/test_sonar.py`)

---

## Dependencies

```
smbus2      # I2C for PCA9685
RPi.GPIO    # GPIO + software PWM for MotorShield
```

Planned additions: `opencv-python`, `mcp` (Anthropic), `flotilla` (Pimoroni — for Flotilla dock + Motion/Weather/Colour modules).

Enable I2C: `raspi-config` → Interface Options → I2C.
Disable SPI (default) to free MotorShield pins for motors 3 and 4.

---

## Gotchas

- `GPIOMotor` does not call `GPIO.setmode()` — caller must. `RobotArm` handles this.
- `I2CMotor.close_all()` closes every PCA9685 in the process, not just one motor's.
- `TrackedBase.close()` calls `close_all()` — don't use if other I2CMotors must stay open.
- Software PWM via RPi.GPIO is jittery under load. Consider hardware PWM for arm precision.
- MotorShield vendor library uses `GPIO.BCM` — do not import it alongside this codebase.
- `min_speed` in `MotorConfig` means any non-zero request is raised to at least this value.
  Motors have a stall speed below which they draw current but don't turn.
