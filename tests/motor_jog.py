"""
Single-motor jog utility for ARGOS.

Interactive:   python3 tests/motor_jog.py
With args:     python3 tests/motor_jog.py <type> <id> <speed> <duration>

  type     — i2c or gpio
  id       — motor ID (i2c: 0-1, gpio: 1-4)
  speed    — -100 to 100
  duration — seconds (0.1 to 10)

Example:  python3 tests/motor_jog.py gpio 4 50 1.0
"""

import sys
import time
import argparse

sys.path.insert(0, "/home/zimchaa/argos")

import RPi.GPIO as GPIO
from argos.drivers.pca9685 import I2CMotor
from argos.drivers.gpio_motor import GPIOMotor
from argos.config import TRACK_MOTORS, ARM_JOINTS

# Build lookup: (type, id) → label for display
_LABELS = {("i2c", cfg.motor_id): label for label, cfg in TRACK_MOTORS.items()}
_LABELS.update({("gpio", cfg.motor_id): name for name, cfg in ARM_JOINTS.items()})


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

    print("ARGOS — motor jog")
    print("  I2C motors (tracks):")
    for label, cfg in TRACK_MOTORS.items():
        print(f"    {label:8s}  id={cfg.motor_id}  +={cfg.positive}  -={cfg.negative}")
    print("  GPIO motors (arm):")
    for label, cfg in ARM_JOINTS.items():
        print(f"    {label:8s}  id={cfg.motor_id}  +={cfg.positive}  -={cfg.negative}")
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
        duration   = ask("Duration seconds (0.1 to 10)", cast=float, validate=lambda v: 0.1 <= v <= 10)

    label = _LABELS.get((motor_type, motor_id), f"motor {motor_id}")
    print(f"\n  Running {motor_type.upper()} motor {motor_id} ({label}) at {speed}% for {duration}s ...")

    try:
        if motor_type == "i2c":
            m = I2CMotor(motor_id=motor_id)
            m.run(speed)
            time.sleep(duration)
            m.stop()
            I2CMotor.close_all()
        else:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            m = GPIOMotor(motor_id)
            m.run(speed)
            time.sleep(duration)
            m.cleanup()
            GPIO.cleanup()
        print("  Done.")
    except KeyboardInterrupt:
        print("\n  Aborted.")
        try:
            I2CMotor.close_all()
        except Exception:
            pass
        try:
            GPIO.cleanup()
        except Exception:
            pass
