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
docs/               # Hardware datasheets and schematics
  sbcomponents_motorshield_schematic_v1.3.pdf
  waveshare_motor_driver_hat_schematic.pdf
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

## Waveshare Motor Driver HAT — full board features

Schematic: `docs/waveshare_motor_driver_hat_schematic.pdf`

### ICs
| IC | Part | Function |
|----|------|----------|
| PCA1 | PCA9685 | 16-channel PWM controller — I2C 0x40 |
| TB1 | TB6612FNG | Dual H-bridge motor driver |
| U1 | MP1584 | Step-down switcher — generates 5V from VIN |
| U2 | RT9193-33 | 3.3V LDO from 5V |

The Pi communicates **only over I2C** (SDA/SCL). The PCA9685 generates all PWM and
direction signals; no other Pi GPIO pins are used by this HAT.

### PCA9685 channel usage
| Channel | Signal | Used for |
|---------|--------|----------|
| 0 | PWMA | Motor 0 speed (left track) |
| 1 | AIN2 | Motor 0 direction |
| 2 | AIN1 | Motor 0 direction |
| 3 | BIN2 | Motor 1 direction |
| 4 | BIN1 | Motor 1 direction |
| 5 | PWMB | Motor 1 speed (right track) |
| **6–15** | — | **FREE** — unconnected on this board |

Channels 6–15 are brought out to the PCA9685 pads but not wired to anything.
They are accessible over I2C at 0x40 and can drive servos, LEDs, or additional
motor drivers wired externally.

### I2C address configuration
A0–A4 address pins have unpopulated solder-pad resistors (R8–R12, footprint NC).
Populate any combination to change the address from 0x40 up to 0x7F.
Allows stacking a second Waveshare HAT at a different address for more motors.

### Headers
| Header | Pins | Purpose |
|--------|------|---------|
| MOTOR1 | 2-pin | Motor A output (A1, A2) |
| MOTOR2 | 2-pin | Motor B output (B1, B2) |
| H1 | 6-pin | VIN, GND, A1, A2, B1, B2 combined |
| VIN1 | 2-pin | External motor power input |
| S1 | 6-pin | Power switch |
| P? | 5-pin | **I2C expansion: 5V, 3V3, GND, SDA, SCL** |

The 5-pin I2C expansion header (P?) exposes the I2C bus directly — ideal for
attaching an IMU (e.g. MPU-6050 at 0x68, no address conflict with 0x40).

### Power
- Motor power: external VIN via VIN1/H1
- 5V logic: generated onboard by MP1584 switcher from VIN
- 3.3V: RT9193-33 LDO — powers PCA9685 and I2C pull-ups
- PWR1: power indicator LED

---

## SB Components MotorShield — full board features

Schematic: `docs/sbcomponents_motorshield_schematic_v1.3.pdf` (v1.3, dated 11/10/2016)
Motor ICs: 2× L293DD dual H-bridge. Schematic source: github.com/sbcshop/MotorShield

### Features beyond motor drive

#### Direction indicator LEDs (already on board, no extra hardware needed)
21 LEDs (L1–L21) in 4 groups driven by NPN transistors T1–T4 (BC817).
Each transistor is driven by a dedicated Pi GPIO pin via a 1K base resistor,
with a 1K collector resistor to +5V feeding the LED group.

| GPIO (BOARD) | BCM   | Drives         |
|--------------|-------|----------------|
| 33           | GPIO13| T2 — LED group (motors 1+2 direction A) |
| 35           | GPIO19| T1 — LED group (motors 1+2 direction B) |
| 36           | GPIO16| T3 — LED group (motors 3+4 direction A) |
| 37           | GPIO26 | T4 — LED group (motors 3+4 direction B) |

These pins are currently **undriven** in our code — the LEDs are dark.
Future: drive these in software to give visual direction feedback.

#### Ultrasonic sensor header (CN10 — 4-pin, not populated)
Voltage divider already fitted on board (R10 1K / R14 2K) to level-shift
the 5V ECHO signal to 3.3V safe for the Pi. Just needs a sensor plugged in.

| Pin function | GPIO (BOARD) | BCM   |
|--------------|--------------|-------|
| TRIG (output)| 29           | GPIO5 |
| ECHO (input) | 31           | GPIO6 |

#### IR sensor headers (CN8, CN9 — 3-pin each, not populated)
Voltage dividers already fitted (R8/R15 and R9/R13 respectively).

| Sensor | GPIO (BOARD) | BCM    |
|--------|--------------|--------|
| IR1 (CN9) | 7        | GPIO4  |
| IR2 (CN8) | 12       | GPIO18 |

#### Power connector (CN6, CN7)
12V motor supply input with reverse-polarity protection diode D1 and
100µF/16V bulk decoupling capacitor C5.

### Complete GPIO pin usage (BOARD numbering)

| BOARD | BCM    | Used by                          |
|-------|--------|----------------------------------|
| 3     | GPIO2  | I2C SDA (Waveshare HAT)          |
| 5     | GPIO3  | I2C SCL (Waveshare HAT)          |
| 7     | GPIO4  | IR1 signal (CN9, unpopulated)    |
| 11    | GPIO17 | Motor 1 enable (gripper)         |
| 12    | GPIO18 | IR2 signal (CN8, unpopulated)    |
| 13    | GPIO27 | Motor 1 pin_b (gripper)          |
| 15    | GPIO22 | Motor 1 pin_a (gripper)          |
| 16    | GPIO23 | Motor 2 pin_a (elbow)            |
| 18    | GPIO24 | Motor 2 pin_b (elbow)            |
| 19    | GPIO10 | Motor 3 enable (wrist)           |
| 21    | GPIO9  | Motor 3 pin_a (wrist)            |
| 22    | GPIO25 | Motor 2 enable (elbow)           |
| 23    | GPIO11 | Motor 3 pin_b (wrist)            |
| 24    | GPIO8  | Motor 4 pin_a (shoulder)         |
| 26    | GPIO7  | Motor 4 pin_b (shoulder)         |
| 29    | GPIO5  | Ultrasonic TRIG (CN10, unpopulated) |
| 31    | GPIO6  | Ultrasonic ECHO (CN10, unpopulated) |
| 32    | GPIO12 | Motor 4 enable (shoulder)        |
| 33    | GPIO13 | Direction LED group T2           |
| 35    | GPIO19 | Direction LED group T1           |
| 36    | GPIO16 | Direction LED group T3           |
| 37    | GPIO26 | Direction LED group T4           |
| **38**| GPIO20 | **FREE**                         |
| **40**| GPIO21 | **FREE**                         |

GND available at BOARD 39 (adjacent to 38 and 40).

> **Note**: BOARD 33 (GPIO13) was previously suggested for an external LED —
> this is incorrect, it is wired to direction LEDs on the shield.
> Use **BOARD 38 + 39** or **BOARD 40 + 39** for any external LED.

---

## Hardware shopping list

Components identified as useful but not yet purchased:

| Priority | Item | Purpose | Notes |
|----------|------|---------|-------|
| 1 | MPU-6050 IMU | Gyro + accelerometer for odometry / turn angle | Unblocks base navigation. I2C at 0x68 — no conflict with 0x40. Plugs into Waveshare I2C expansion header (P?, 5-pin). ~£3 |
| 2 | HC-SR04 ultrasonic sensor | Obstacle detection / safety | Needed before any autonomous movement. Plugs into CN10 on MotorShield — voltage divider already fitted. TRIG=BOARD 29 (GPIO5), ECHO=BOARD 31 (GPIO6) |
| 3 | IR sensors ×2 | Line following / object detection | Plugs into CN8/CN9 on MotorShield — voltage dividers already fitted. IR1=BOARD 7 (GPIO4), IR2=BOARD 12 (GPIO18) |
| 4 | Servo(s) | Camera pan/tilt or additional DOF | Drive via free PCA9685 channels 6–15 on Waveshare HAT — no extra hardware needed |
| 5 | 330Ω resistor | Current-limiting for external LED | Connect LED between BOARD 38 (GPIO20) + BOARD 39 (GND). BOARD 33 is NOT free — drives onboard direction LEDs on MotorShield |

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
- Both tracks spin forward on positive speed; `TrackedBase.forward()` drives both together
- Elbow, wrist, gripper all respond correctly via GPIOMotor driver

**Known issues:**
- **Track drift**: motor 1 (right) runs mechanically slower than motor 0 at equal power.
  Will need feedback (IMU/encoder/visual odometry) to compensate automatically.
- **Shoulder not moving**: motor 4 terminal produced no movement. Investigate connector and mechanical state before using the arm.

**Arm wiring (confirmed on hardware 2026-02-24):**
| Joint    | MotorShield terminal |
|----------|----------------------|
| shoulder | 4                    |
| elbow    | 2                    |
| wrist    | 3                    |
| gripper  | 1                    |

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
