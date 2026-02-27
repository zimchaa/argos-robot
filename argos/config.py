"""
Central hardware configuration for ARGOS.

All hardware mappings live here — motors, sensors, I2C addresses, and
Flotilla dock assignments.  Import from this module rather than
hardcoding anything in drivers, controllers, or test scripts.
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

# ---------------------------------------------------------------------------
# Sensorium — I2C addresses and Flotilla dock channel assignments
# ---------------------------------------------------------------------------

# MPU-6050 inertial measurement unit (gyro + accel)
IMU_I2C_BUS     = 1       # Raspberry Pi I2C bus
IMU_I2C_ADDR    = 0x68    # default MPU-6050 address (AD0 low)

# Flotilla Motion module dock ports
# ch 6 — body-mounted LSM303D (accel + mag); used for AHRS heading
# ch 1 — shoulder-to-elbow arm link LSM303D; used for joint angle estimation
FLOTILLA_BODY_MOTION_CH = 6
FLOTILLA_ARM_MOTION_CH  = 1

# HC-SR04 ultrasonic (BOARD pin numbers; voltage divider fitted on CN10)
SONAR_TRIG_PIN  = 29
SONAR_ECHO_PIN  = 31

# IR proximity sensors (BOARD pin numbers)
IR_PIN_1        = 7    # CN9
IR_PIN_2        = 12   # CN8
