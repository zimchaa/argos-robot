"""
Manual hardware test for all motors.

Run with: python3 tests/test_all_motors_manual.py

Steps:
  1. Comms check — read MODE1 register from PCA9685
  2. Motor 0 (left track) forward for 1 s then stop
  3. Motor 1 (right track) forward for 1 s then stop
  4. Both together (TrackedBase.forward) for 1.5 s then stop
  5. Arm joints — shoulder, elbow, wrist, gripper — each 0.5 s at low speed
     NO governor fitted: speed is capped at 25%, duration 0.5 s. Be ready to Ctrl-C.

Motors will actually spin — make sure joints and tracks are free to move safely.
"""

import sys
import time

sys.path.insert(0, "/home/zimchaa/argos")

import smbus2
import RPi.GPIO as GPIO
from argos.drivers.pca9685 import PCA9685, I2CMotor
from argos.drivers.gpio_motor import GPIOMotor
from argos.base.tracks import TrackedBase

ARM_SPEED    = 25   # conservative — no governor fitted
ARM_DURATION = 0.5  # seconds


def check_comms():
    print("1. Comms check — reading MODE1 from PCA9685 at 0x40 ...")
    bus = smbus2.SMBus(1)
    val = bus.read_byte_data(0x40, 0x00)
    bus.close()
    print(f"   MODE1 = 0x{val:02X}  (expect 0x00 or 0x20 after reset)  ✓")


def test_motor(motor_id, label, speed=70, duration=1.0):
    print(f"\n2. Motor {motor_id} ({label}) — {speed}% for {duration}s ...")
    m = I2CMotor(motor_id=motor_id)
    m.run(speed)
    time.sleep(duration)
    m.stop()
    I2CMotor.close_all()
    print(f"   Motor {motor_id} stopped  ✓")


def test_base(speed=70, duration=1.5):
    print(f"\n3. TrackedBase.forward({speed}) for {duration}s ...")
    base = TrackedBase()
    base.forward(speed)
    time.sleep(duration)
    base.stop()
    base.close()
    print("   Base stopped  ✓")


def test_joint(motor_id, label):
    print(f"\n   Joint {motor_id} ({label}) — +{ARM_SPEED}% for {ARM_DURATION}s ...")
    m = GPIOMotor(motor_id)
    m.run(ARM_SPEED)
    time.sleep(ARM_DURATION)
    m.cleanup()
    print(f"   {label} stopped  ✓  — note which direction it moved")


if __name__ == "__main__":
    try:
        # --- Tracks ---
        check_comms()
        input("\n   Press Enter to spin Motor 0 (left) ...")
        test_motor(0, "left")
        input("   Press Enter to spin Motor 1 (right) ...")
        test_motor(1, "right")
        input("   Press Enter to run both tracks forward ...")
        test_base()

        # --- Arm joints ---
        print("\n-- ARM JOINTS (SB Components MotorShield) --")
        print(f"   Speed capped at {ARM_SPEED}%, duration {ARM_DURATION}s — no governor fitted.")
        print("   Note which direction each joint moves for +ve speed.")
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        for motor_id, label in [(1, "shoulder"), (2, "elbow"), (3, "wrist"), (4, "gripper")]:
            input(f"   Press Enter to jog {label} ...")
            test_joint(motor_id, label)
        GPIO.cleanup()

        print("\nAll tests passed.")
    except KeyboardInterrupt:
        print("\nAborted — cleaning up ...")
        try:
            I2CMotor.close_all()
        except Exception:
            pass
        try:
            GPIO.cleanup()
        except Exception:
            pass
    except Exception as e:
        print(f"\nERROR: {e}")
        try:
            GPIO.cleanup()
        except Exception:
            pass
        raise
