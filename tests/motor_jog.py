"""
Single-motor jog utility for ARGOS.

Run with: python3 tests/motor_jog.py

Asks for motor type, ID, speed, and duration, then runs that one motor.
Useful for spot-checking individual motors without running the full test suite.
"""

import sys
import time

sys.path.insert(0, "/home/zimchaa/argos")

import RPi.GPIO as GPIO
from argos.drivers.pca9685 import I2CMotor
from argos.drivers.gpio_motor import GPIOMotor
from argos.config import TRACK_MOTORS, ARM_JOINTS

# Build lookup: (type, id) → label for display
_LABELS = {("i2c", mid): label for mid, label in TRACK_MOTORS.items()}
_LABELS.update({("gpio", mid): name for name, mid in ARM_JOINTS.items()})


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
    print("ARGOS — motor jog")
    print(f"  I2C motors (tracks):  {TRACK_MOTORS}")
    print(f"  GPIO motors (arm):    {ARM_JOINTS}")
    print()

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
