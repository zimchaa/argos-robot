"""
Central hardware configuration for ARGOS.

All motor ID mappings live here. Import from this module rather than
hardcoding IDs in drivers, controllers, or test scripts.
"""

from dataclasses import dataclass


@dataclass
class MotorConfig:
    motor_id: int
    positive: str        # physical effect of +ve speed
    negative: str        # physical effect of -ve speed
    max_speed: int = 100 # hard upper cap (0–100); motor never exceeds this
    min_speed: int = 0   # minimum effective speed; non-zero requests clamped up to this
    max_duration: float = 30.0  # seconds before watchdog auto-stops the motor


# ---------------------------------------------------------------------------
# Tracks — Waveshare Motor Driver HAT (I2C 0x40, bus 1)
# ---------------------------------------------------------------------------
TRACK_MOTORS = {
    "left":  MotorConfig(0, "forward",  "backward", max_speed=100, min_speed=30, max_duration=10.0),
    "right": MotorConfig(1, "forward",  "backward", max_speed=100, min_speed=30, max_duration=10.0),
}

# ---------------------------------------------------------------------------
# Arm joints — SB Components MotorShield (GPIO BOARD pins)
# Wiring confirmed on hardware 2026-02-24.
# ---------------------------------------------------------------------------
ARM_JOINTS = {
    "shoulder": MotorConfig(4, "raise",   "lower",   max_speed=80, min_speed=25, max_duration=3.0),
    "elbow":    MotorConfig(2, "extend",  "retract", max_speed=80, min_speed=25, max_duration=3.0),
    "wrist":    MotorConfig(3, "raise",   "lower",   max_speed=80, min_speed=25, max_duration=3.0),
    "gripper":  MotorConfig(1, "open",    "close",   max_speed=80, min_speed=20, max_duration=2.0),
}
