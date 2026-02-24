# ARGOS Robot — Claude Context

## Project overview

ARGOS is a Python robotics control library for a Raspberry Pi robot. It has two
subsystems: a two-track differential-drive base and a four-joint articulated arm.
The code provides hardware abstraction drivers plus high-level motion controllers.

Target hardware: Raspberry Pi 4 (hostname `argos.local`).
Development platform: Raspberry Pi 400.

---

## Repository layout

```
argos/
  drivers/
    pca9685.py      # PCA9685 I2C driver + I2CMotor (tracks, via Waveshare HAT)
    gpio_motor.py   # GPIOMotor (arm joints, via SB Components MotorShield)
  base/
    tracks.py       # TrackedBase — high-level differential drive controller
    odometry.py     # [planned] Dead-reckoning / sensor-based base odometry
  arm/
    joints.py       # RobotArm — high-level arm controller (4 named joints)
    kinematics.py   # [planned] Forward + inverse kinematics
  vision/
    camera.py       # [planned] Frame capture
    aruco.py        # [planned] ArUco detection, joint angle extraction
    calibration/    # [planned] Camera intrinsics + distortion data
  safety/
    monitor.py      # [planned] Background safety watchdog
    limits.py       # [planned] Joint angle limits, workspace bounds
  mcp/
    server.py       # [planned] MCP tool definitions + server
    __main__.py     # [planned] python -m argos.mcp entry point
requirements.txt    # smbus2, RPi.GPIO (+ opencv-python, mcp planned)
DEVLOG.md           # Session-by-session development log
```

---

## Architecture

Two layers:

**Drivers** (low-level, hardware-specific):
- `I2CMotor` — single DC motor on the Waveshare Motor Driver HAT. Communicates
  over I2C using a shared `PCA9685` instance per chip address. Motors on the same
  chip share one `smbus2` connection and are only initialised once.
- `GPIOMotor` — single DC motor on the SB Components MotorShield. Uses
  `RPi.GPIO` software PWM. Requires `GPIO.setmode(GPIO.BOARD)` before use.

**Controllers** (high-level, subsystem-level):
- `TrackedBase` — wraps two `I2CMotor` instances with named drive commands.
- `RobotArm` — wraps four `GPIOMotor` instances as named joint attributes.

Both motor types expose an identical `run(speed)` interface. Speed is a signed
integer **-100 to 100** (negative = reverse, 0 = stop, magnitude = duty cycle
percent). There are no direction strings anywhere in the API.

---

## Hardware mapping

### Tracks — Waveshare Motor Driver HAT (I2C address 0x40, bus 1)

The HAT uses a TB6612FNG driver IC driven by a PCA9685 PWM controller.

| Motor ID | Track   | PCA9685 channels        |
|----------|---------|-------------------------|
| 0        | Left    | PWM=ch0, IN1=ch1, IN2=ch2 |
| 1        | Right   | PWM=ch5, IN1=ch3, IN2=ch4 |

Direction encoding (matches vendor `MotorDriver.py`):
- Positive speed → IN1=0, IN2=1
- Negative speed → IN1=1, IN2=0

### Arm — SB Components MotorShield (GPIO, no I2C)

| Motor ID | Joint    | enable | pin_a | pin_b |
|----------|----------|--------|-------|-------|
| 1        | shoulder | 11     | 15    | 13    |
| 2        | elbow    | 22     | 16    | 18    |
| 3        | wrist    | 19     | 21    | 23    |
| 4        | gripper  | 32     | 24    | 26    |

All pin numbers are BOARD-numbered. Motors 3 and 4 use SPI pins — safe when SPI
is disabled (default on Pi OS Lite).

To reverse a joint's effective direction, swap its `pin_a`/`pin_b` values in
`gpio_motor._MOTOR_PINS`. Do not add direction logic elsewhere.

---

## Key conventions

- **Speed range**: always -100..100; never pass raw PWM counts or fractions to
  public APIs. Both driver classes clamp the input.
- **GPIO mode**: always `GPIO.BOARD`. `RobotArm.__init__` calls
  `GPIO.setmode(GPIO.BOARD)`. Do not mix in `GPIO.BCM` anywhere.
- **PCA9685 sharing**: `I2CMotor` uses a module-level `_pca_instances` dict keyed
  by `(bus, address)`. Creating multiple `I2CMotor` instances for the same chip
  reuses the existing connection; don't bypass this.
- **Shutdown**: call `TrackedBase.close()` (zeroes channels, closes smbus) and
  `RobotArm.cleanup()` (stops PWM, calls `GPIO.cleanup()`) before exiting.
- **No vendor libraries imported**: both drivers are written from scratch with
  `smbus2` and `RPi.GPIO` only. The vendor sources were read for register/pin
  information but are not a dependency.

---

## Dependencies

```
smbus2      # I2C communication for PCA9685
RPi.GPIO    # GPIO and software PWM for MotorShield
```

Install on the Pi: `pip install -r requirements.txt`

I2C must be enabled on the Pi (`raspi-config` → Interface Options → I2C).
SPI should be **disabled** (default) to free pins for Motors 3 and 4.

---

## Planned system architecture (Phase 2+)

The end goal is to expose the robot to a remote multimodal LLM (e.g. Claude)
via an MCP server running on the Pi. The LLM issues high-level commands; the Pi
handles vision, kinematics, safety, and motor execution locally.

### Layer stack (bottom to top)

```
┌─────────────────────────────────────────────────────────┐
│  Remote LLM (Claude via API)                            │
│  — sees camera frames, issues semantic commands         │
├─────────────────────────────────────────────────────────┤
│  MCP server  argos/mcp/server.py                        │
│  — exposes tools over stdio/SSE to the LLM client       │
├─────────────────────────────────────────────────────────┤
│  Motion planner  argos/arm/kinematics.py                │
│                  argos/base/odometry.py                 │
│  — IK for arm pose targets, dead-reckoning for base     │
├─────────────────────────────────────────────────────────┤
│  Safety layer  argos/safety/monitor.py        ◄─ AUTH   │
│  — joint angle limits, visual bounds check              │
│  — can hard-stop any motor, overrides all layers above  │
├─────────────────────────────────────────────────────────┤
│  Vision  argos/vision/camera.py                         │
│          argos/vision/aruco.py                          │
│  — frame capture, ArUco detection, arm pose estimation  │
├─────────────────────────────────────────────────────────┤
│  Controllers (Phase 1 — already built)                  │
│  TrackedBase  /  RobotArm                               │
├─────────────────────────────────────────────────────────┤
│  Drivers (Phase 1 — already built)                      │
│  I2CMotor  /  GPIOMotor                                 │
└─────────────────────────────────────────────────────────┘
```

The safety layer has **unconditional authority**: it can stop motors regardless
of what any higher layer requests. All motion above driver level must pass
through or be monitored by it.

---

## Vision subsystem (planned)

### Camera
- Mounted fixed on the robot body, pointing at the arm.
- Likely: Raspberry Pi Camera Module (picamera2 library) or USB webcam (OpenCV
  VideoCapture). Final choice deferred to hardware phase.

### Arm pose tracking
- **ArUco markers** (OpenCV `cv2.aruco`) affixed to each arm link.
- One marker per link gives 6-DOF pose of that segment relative to the camera.
- Adjacent marker poses are compared to derive joint angles.
- Markers serve a dual purpose: pose tracking + visual reference for the LLM.
- Camera must be calibrated (intrinsic matrix + distortion coefficients) for
  accurate `solvePnP`-based pose estimation. Calibration stored in a config
  file, not hard-coded.

### Frame delivery to LLM
- `get_camera_frame()` MCP tool returns a JPEG-encoded frame as base64.
- The LLM (multimodal) receives this and can reason about the scene before
  issuing commands.

### Planned modules
```
argos/vision/
  camera.py     # Frame capture, resolution/fps config
  aruco.py      # Marker detection, joint angle extraction
  calibration/  # Camera calibration data (intrinsics, distortion)
```

---

## Arm kinematics (planned)

### Joint configuration
Vertical-plane serial chain: all joints rotate in the same plane.

```
         [wrist]──[gripper]
        /
[shoulder]──[elbow]
   |
  base (fixed)
```

- **Shoulder**: lifts/lowers the entire arm (rotation in vertical plane)
- **Elbow**: bends the forearm segment
- **Wrist**: tilts the end effector
- **Gripper**: opens/closes (binary open/close or speed-controlled)

### Inverse kinematics
With a planar 3-link arm (shoulder + elbow + wrist in one plane), IK reduces
to a 2D problem: given a target end-effector position (r, z) and orientation,
solve for shoulder and elbow angles analytically, then wrist compensates.
Exact link lengths TBD — to be measured on hardware and stored as constants.

### Planned modules
```
argos/arm/
  kinematics.py   # Forward + inverse kinematics, link length constants
```

---

## Safety layer (planned)

Responsibilities:
- Define per-joint **angle limits** (configurable, stored in a config file or
  dataclass — not scattered through the code).
- Run as a background monitor (thread or asyncio task) that reads current arm
  pose from the vision layer and calls `RobotArm.stop()` if any joint exceeds
  its limit.
- Expose an **emergency stop** callable that higher layers can also trigger.
- The safety layer is the only place that may call `stop()` proactively —
  higher layers request motion; the safety layer decides whether to allow it.

```
argos/safety/
  monitor.py    # SafetyMonitor — background watchdog + limit enforcement
  limits.py     # Joint angle limits, workspace bounds (constants/config)
```

---

## MCP interface (planned)

The Pi runs an MCP server that the remote LLM connects to. Tools expose semantic
actions; the server translates them to kinematics + motor commands.

### Planned tools

| Tool | Arguments | Returns | Notes |
|------|-----------|---------|-------|
| `get_camera_frame` | — | base64 JPEG string | For LLM visual context |
| `get_arm_pose` | — | joint angles + end-effector (x,y,z) | Current state |
| `move_arm` | `x, y, z, grip` | success / error | IK → motor execution with visual feedback |
| `move_base_forward` | `cm: float` | success / error | Dead-reckoning (see odometry section) |
| `turn_base` | `degrees: float` | success / error | Positive = right, negative = left |
| `stop_all` | — | — | Emergency stop, overrides everything |

### Protocol
- MCP transport: stdio (local) or SSE (network). Network SSE preferred so the
  LLM client can be on a separate machine.
- Library: Anthropic `mcp` Python package.
- Server entry point: `argos/mcp/server.py`, runnable as `python -m argos.mcp`.

```
argos/mcp/
  server.py     # Tool definitions, MCP server startup
  __main__.py   # python -m argos.mcp entry point
```

---

## Base odometry (open question)

The tracks have no encoders. Options under consideration — a sensor shopping
list for future hardware decisions:

| Sensor | Interface | Gives | Notes |
|--------|-----------|-------|-------|
| MPU-6050 | I2C (0x68) | Gyro (turn angle) + accel | ~£3, very common Pi add-on |
| BNO055 | I2C | Full 9-DOF with sensor fusion (yaw/pitch/roll) | More accurate, ~£10 |
| Wheel encoders | GPIO interrupt | Precise distance + turn | Requires mechanical attachment to tracks |
| HC-SR04 ultrasonic | GPIO trigger/echo | Obstacle distance (not odometry) | Useful for safety, not navigation |
| Visual odometry | Camera | Position from environmental landmarks | Software only; camera currently faces arm not forward |

**Current approach**: defer. `move_base_forward` and `turn_base` will initially
use time-based dead reckoning (speed × time = approximate distance/angle).
Accuracy will be assessed on hardware; a sensor will be added if needed.
An IMU (MPU-6050) is the most likely addition — cheap, I2C (bus 1 is already
in use), and gives reliable turn angle.

---

## Development status

Phase 1 code complete. Hardware bring-up in progress (Session 2).

**Confirmed working:**
- Waveshare Motor Driver HAT present at I2C 0x40 (`i2cdetect -y 1` verified)
- `smbus2` and `RPi.GPIO` installed on target Pi 4
- Both tracks (Motor 0 left, Motor 1 right) spin correctly; positive speed = forward
- `TrackedBase.forward()` drives both tracks together

**Still to verify:**
- Arm joint directions — run `tests/test_all_motors_manual.py` and note which way each joint moves for +ve speed; swap `pin_a`/`pin_b` in `gpio_motor._MOTOR_PINS` for any that are reversed
- GPIO pin conflicts between stacked HATs (I2C side looks clean; GPIO TBD)
- No governor / joint angle limits yet — arm testing must be done at low speed

**No automated test runner configured.** Manual test script: `tests/test_all_motors_manual.py`

When hardware testing begins, update DEVLOG.md with findings.

---

## Running / deploying

There is no build step. Deploy by copying the repo to the Pi and installing
dependencies:

```bash
pip install -r requirements.txt
```

Typical usage pattern:

```python
from argos.base.tracks import TrackedBase
from argos.arm.joints import RobotArm

base = TrackedBase()
arm  = RobotArm()

try:
    base.forward(60)
    arm.shoulder.run(30)
    # ...
finally:
    base.close()
    arm.cleanup()
```

---

## Things to watch out for

- `GPIOMotor` does **not** call `GPIO.setmode()` — the caller must do it first.
  `RobotArm` handles this; if using `GPIOMotor` directly, set the mode manually.
- `I2CMotor.close_all()` closes **every** PCA9685 connection in the process,
  not just the one for a specific motor.
- `TrackedBase.close()` calls `I2CMotor.close_all()` — don't call it if other
  I2CMotor instances need to stay open.
- Software PWM via `RPi.GPIO` can be jittery under CPU load. If precision
  matters for arm control, consider hardware PWM or a second PCA9685.
- The MotorShield vendor library uses `GPIO.BCM` internally — do not import or
  use it alongside this codebase.
