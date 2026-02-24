"""
High-level robot arm controller for ARGOS.
Wraps four GPIOMotor instances with named joint attributes.

Joint → MotorShield motor ID:
  shoulder → 4  (confirmed on hardware 2026-02-24; motor 4 terminal)
  elbow    → 2  (confirmed on hardware 2026-02-24)
  wrist    → 3  (confirmed on hardware 2026-02-24)
  gripper  → 1  (confirmed on hardware 2026-02-24; motor 1 terminal)

Speed is -100 to 100. Which physical direction is positive is determined by
wiring. To reverse a joint, swap its pin_a/pin_b in gpio_motor._MOTOR_PINS.
"""

import RPi.GPIO as GPIO
from argos.drivers.gpio_motor import GPIOMotor


class RobotArm:
    """
    Controller for the ARGOS robot arm (4 joints via MotorShield GPIO).
    Sets GPIO mode on construction; call cleanup() on shutdown.
    """

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        self.shoulder = GPIOMotor(4)
        self.elbow    = GPIOMotor(2)
        self.wrist    = GPIOMotor(3)
        self.gripper  = GPIOMotor(1)
        self._joints  = [self.shoulder, self.elbow, self.wrist, self.gripper]

    def stop(self):
        for joint in self._joints:
            joint.stop()

    def cleanup(self):
        self.stop()
        for joint in self._joints:
            joint.cleanup()
        GPIO.cleanup()
