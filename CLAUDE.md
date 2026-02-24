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
  arm/
    joints.py       # RobotArm — high-level arm controller (4 named joints)
requirements.txt    # smbus2, RPi.GPIO
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

## Development status

As of the last session (Phase 1), all code is written and structured but
**no hardware testing has been done yet**. Specifically:

- `i2cdetect -y 1` has not been run to confirm the Waveshare HAT appears at 0x40
- GPIO pin conflicts between the two stacked HATs are unconfirmed on hardware
- `smbus2` and `RPi.GPIO` are not yet installed on the target Pi 4
- No tests exist; the project has no test runner configured

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
