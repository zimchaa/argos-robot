"""
Sensor axis alignment probe for ARGOS.

Prints raw accel and mag readings from the MPU-6050 and from each Flotilla
Motion module side by side, at ~4 Hz.  Use this to determine the axis mapping
between sensors so the Madgwick MARG filter can receive aligned inputs.

Procedure
---------
1.  Run with the robot on a flat, level surface.
    Identify which axis reads ~+1.0 g for each sensor (should be Z-up).

2.  Tilt the robot NOSE UP (front of chassis rises).
    Identify which axis gains positive g on each sensor.  That axis is
    the sensor's "pitch" axis.

3.  Tilt the robot RIGHT SIDE DOWN.
    Identify which axis gains positive g.  That is the "roll" axis.

4.  Slowly rotate the robot in a flat circle (yaw) and watch the mag
    values change on the body Motion module (ch 6).  Note which two axes
    trace out a near-circle (those are the horizontal mag components).

Record the results and use them to fill in the MAG_REMAP tuple in
test_ahrs.py / ahrs.py once the mapping is confirmed.

Usage:
    python3 tests/probe_sensor_axes.py

Channels
--------
ch 1  — shoulder-to-elbow arm sensor
ch 6  — body-mounted sensor (used for MARG heading)
"""

import sys
import pathlib
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from argos.sensorium.imu import MPU6050
from argos.sensorium.flotilla import FlotillaReader
from argos.config import FLOTILLA_BODY_MOTION_CH, FLOTILLA_ARM_MOTION_CH

RATE_HZ = 4
DT      = 1.0 / RATE_HZ

ARM_CH  = FLOTILLA_ARM_MOTION_CH
BODY_CH = FLOTILLA_BODY_MOTION_CH


def _bar(v, scale=1.0, width=7):
    """Simple ASCII bar: fill proportional to v/scale."""
    frac = max(-1.0, min(1.0, v / scale))
    half = width // 2
    filled = round(abs(frac) * half)
    if frac >= 0:
        return " " * half + "#" * filled + " " * (half - filled)
    else:
        return " " * (half - filled) + "#" * filled + " " * half


def _fmt_acc(ax, ay, az):
    return (f"ax={ax:+6.3f}g  ay={ay:+6.3f}g  az={az:+6.3f}g")


def _fmt_mag(mx, my, mz):
    return (f"mx={mx:+6d}  my={my:+6d}  mz={mz:+6d}")


def main():
    print("ARGOS — sensor axis probe  (Ctrl-C to stop)\n")

    imu = MPU6050()
    print("  MPU-6050 opened at I2C 0x68")

    flotilla = FlotillaReader()
    flotilla.start()
    print("  Flotilla dock opened — waiting 1 s for modules ...")
    time.sleep(1.0)

    chs = flotilla.connected_modules
    print(f"  Connected modules: {chs}\n")

    arm_present  = ARM_CH  in chs
    body_present = BODY_CH in chs
    if not body_present:
        print(f"  WARNING: body sensor not seen on ch {BODY_CH}")
    if not arm_present:
        print(f"  INFO: arm sensor not seen on ch {ARM_CH} (not required)")

    sep = "  " + "-" * 76

    print("  Tilt the robot and watch which axes change.\n")
    print(f"  {'MPU-6050 accel':^26}  {'Body LSM303D (ch6) accel':^26}  "
          f"{'Body LSM303D (ch6) mag':^22}")
    print(sep)

    try:
        while True:
            t0 = time.monotonic()

            r = imu.read()
            body = flotilla.motion_channel(BODY_CH)
            arm  = flotilla.motion_channel(ARM_CH)

            # --- MPU-6050 ---
            imu_line = _fmt_acc(r.accel_x_g, r.accel_y_g, r.accel_z_g)

            # --- Body LSM303D accel ---
            if body is not None:
                body_acc = _fmt_acc(body.acc_x_g, body.acc_y_g, body.acc_z_g)
                body_mag = _fmt_mag(body.mag_x, body.mag_y, body.mag_z)
            else:
                body_acc = "  (no data)              "
                body_mag = "  (no data)          "

            print(f"\r  {imu_line}  |  {body_acc}  |  {body_mag}",
                  end="", flush=True)

            # --- Arm LSM303D (info line, printed on second row) ---
            if arm is not None:
                arm_acc = _fmt_acc(arm.acc_x_g, arm.acc_y_g, arm.acc_z_g)
                arm_head = f"  arm(ch{ARM_CH}): {arm_acc}  "
            else:
                arm_head = f"  arm(ch{ARM_CH}): no data"

            # Print arm below on a separate line each cycle
            print(f"\n{arm_head}", end="")
            print(f"\033[1A", end="", flush=True)   # move cursor up 1 line

            elapsed = time.monotonic() - t0
            if DT - elapsed > 0:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n\nStopped.")
    finally:
        imu.close()
        flotilla.close()


if __name__ == "__main__":
    main()
