"""
Single-motor jog utility for ARGOS — routes through SafetyMonitor.

Interactive:   python3 tests/motor_jog.py
With args:     python3 tests/motor_jog.py <type> <id> <speed> <duration>

  type     — i2c or gpio
  id       — motor ID (i2c: 0-1, gpio: 1-4)
  speed    — -100 to 100 (will be clamped by SafetyMonitor per config limits)
  duration — seconds (if > max_duration for the motor, watchdog stops it early)

Example:  python3 tests/motor_jog.py gpio 4 50 1.0
          python3 tests/motor_jog.py i2c  0 150 3.0   # clamp demo: 150 → 100

Speed clamping is applied by SafetyMonitor before driving the motor.
The reported "clamped" value shows what the motor will actually receive.
"""

import sys
import time
import argparse

sys.path.insert(0, "/home/zimchaa/argos")

from argos.safety.monitor import SafetyMonitor, _clamp_speed
from argos.config import TRACK_MOTORS, ARM_JOINTS

# Map (type, id) → (label, MotorConfig)
_MOTORS = {}
for _label, _cfg in TRACK_MOTORS.items():
    _MOTORS[("i2c", _cfg.motor_id)] = (_label, _cfg)
for _label, _cfg in ARM_JOINTS.items():
    _MOTORS[("gpio", _cfg.motor_id)] = (_label, _cfg)


def ask(prompt, cast=str, validate=None):
    while True:
        raw = input(f"  {prompt}: ").strip()
        try:
            val = cast(raw)
            if validate is None or validate(val):
                return val
            print("  Invalid value, try again.")
        except (ValueError, TypeError):
            print("  Invalid input, try again.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ARGOS single-motor jog")
    parser.add_argument("type",     nargs="?", choices=["i2c", "gpio"])
    parser.add_argument("id",       nargs="?", type=int)
    parser.add_argument("speed",    nargs="?", type=int)
    parser.add_argument("duration", nargs="?", type=float)
    parsed = parser.parse_args()

    print("ARGOS — motor jog (via SafetyMonitor)")
    print("  I2C motors (tracks):")
    for label, cfg in TRACK_MOTORS.items():
        print(f"    {label:8s}  id={cfg.motor_id}"
              f"  max={cfg.max_speed}  min={cfg.min_speed}  dur={cfg.max_duration}s"
              f"  +={cfg.positive}  -={cfg.negative}")
    print("  GPIO motors (arm):")
    for label, cfg in ARM_JOINTS.items():
        print(f"    {label:8s}  id={cfg.motor_id}"
              f"  max={cfg.max_speed}  min={cfg.min_speed}  dur={cfg.max_duration}s"
              f"  +={cfg.positive}  -={cfg.negative}")
    print()

    if all(v is not None for v in (parsed.type, parsed.id, parsed.speed, parsed.duration)):
        motor_type = parsed.type
        motor_id   = parsed.id
        speed      = parsed.speed
        duration   = parsed.duration
    else:
        motor_type = ask("Motor type [i2c/gpio]", validate=lambda v: v in ("i2c", "gpio"))
        motor_id   = ask("Motor ID", cast=int)
        speed      = ask("Speed (-100 to 100)", cast=int, validate=lambda v: -100 <= v <= 100)
        duration   = ask("Duration seconds (0.1 to 30)", cast=float, validate=lambda v: 0.1 <= v <= 30)

    key = (motor_type, motor_id)
    if key not in _MOTORS:
        print(f"  Unknown motor: {motor_type} id={motor_id}")
        sys.exit(1)

    label, cfg = _MOTORS[key]
    clamped    = _clamp_speed(speed, cfg)

    print(f"  Motor  : {motor_type.upper()} id={motor_id} ({label})")
    print(f"  Speed  : {speed}% → clamped to {clamped}%"
          + ("" if speed == clamped else f"  [limit applied]"))
    print(f"  Run for: {duration}s"
          + (f"  [watchdog fires at {cfg.max_duration}s]" if duration > cfg.max_duration else ""))
    print()

    # Runners: call the matching SafeJoint or base motor through the monitor
    def make_runner(safety):
        if motor_type == "i2c":
            motor  = safety._base_ctrl.left if motor_id == 0 else safety._base_ctrl.right
            return lambda spd: safety._checked_run(motor, spd, cfg)
        else:
            joint_name = next(n for n, c in ARM_JOINTS.items() if c.motor_id == motor_id)
            joint = getattr(safety.arm, joint_name)
            return joint.run

    safety = SafetyMonitor()
    try:
        run = make_runner(safety)
        run(speed)
        time.sleep(duration)
        print("  Done.")
    except KeyboardInterrupt:
        print("\n  Aborted.")
    finally:
        safety.close()
