"""
Central hardware configuration for ARGOS.

All motor ID mappings live here. Import from this module rather than
hardcoding IDs in drivers, controllers, or test scripts.
"""

# ---------------------------------------------------------------------------
# Tracks — Waveshare Motor Driver HAT (I2C 0x40, bus 1)
# Motor ID → label
# ---------------------------------------------------------------------------
TRACK_MOTORS = {
    0: "left",
    1: "right",
}

# ---------------------------------------------------------------------------
# Arm joints — SB Components MotorShield (GPIO BOARD pins)
# Joint name → MotorShield motor ID
# Confirmed on hardware 2026-02-24.
# ---------------------------------------------------------------------------
ARM_JOINTS = {
    "shoulder": 4,
    "elbow":    2,
    "wrist":    3,
    "gripper":  1,
}
