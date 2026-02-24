"""
Central hardware configuration for ARGOS.

All motor ID mappings live here. Import from this module rather than
hardcoding IDs in drivers, controllers, or test scripts.
"""

from dataclasses import dataclass


@dataclass
class MotorConfig:
    motor_id: int
    positive: str   # physical effect of +ve speed
    negative: str   # physical effect of -ve speed


# ---------------------------------------------------------------------------
# Tracks — Waveshare Motor Driver HAT (I2C 0x40, bus 1)
# ---------------------------------------------------------------------------
TRACK_MOTORS = {
    "left":  MotorConfig(0, positive="forward",  negative="backward"),
    "right": MotorConfig(1, positive="forward",  negative="backward"),
}

# ---------------------------------------------------------------------------
# Arm joints — SB Components MotorShield (GPIO BOARD pins)
# Wiring confirmed on hardware 2026-02-24.
# ---------------------------------------------------------------------------
ARM_JOINTS = {
    "shoulder": MotorConfig(4, positive="raise",   negative="lower"),
    "elbow":    MotorConfig(2, positive="extend",  negative="retract"),
    "wrist":    MotorConfig(3, positive="raise",   negative="lower"),
    "gripper":  MotorConfig(1, positive="open",    negative="close"),
}
