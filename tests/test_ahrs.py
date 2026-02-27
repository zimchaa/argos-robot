"""
9DOF AHRS live test for ARGOS.

Reads MPU-6050 (gyro + accel) and Flotilla Motion (magnetometer),
runs the Madgwick filter, and prints roll/pitch/yaw continuously.

Startup sequence:
  1. Open MPU-6050 and Flotilla dock.
  2. Calibrate gyro bias (3 s stationary).
  3. Initialise filter from gravity + 1 s level calibration.
  4. Run filter at ~50 Hz, printing orientation.

Usage:
    python3 tests/test_ahrs.py [--imu-only]

    --imu-only   Run 6DOF (no magnetometer). Heading will drift.

Place the robot on a flat, stable surface for gyro calibration.

Sensor roles
------------
MPU-6050 (I2C 0x68)          — gyro + accel for the Madgwick filter
Flotilla Motion ch 6          — body-mounted LSM303D, magnetometer for
                                9DOF heading.  ch 1 is the arm joint
                                sensor and must NOT be used here.
"""

import math
import sys
import pathlib
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from argos.sensorium.imu import MPU6050
from argos.sensorium.ahrs import MadgwickAHRS

RATE_HZ           = 50
DT                = 1.0 / RATE_HZ
CALIB_SECS        = 3
BETA              = 0.05
IMU_ONLY          = "--imu-only" in sys.argv
BODY_MOTION_CH    = 6   # Flotilla dock port for the body-mounted LSM303D


def imu_read_safe(imu):
    """Read IMU with retry on transient I2C errors."""
    for attempt in range(3):
        try:
            return imu.read()
        except OSError:
            time.sleep(0.005)
    return None   # all retries failed


def calibrate_gyro(imu, seconds):
    """Average gyro readings at rest to estimate bias (deg/s)."""
    n = 0
    sx = sy = sz = 0.0
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        r = imu_read_safe(imu)
        if r is None:
            continue
        sx += r.gyro_x_dps
        sy += r.gyro_y_dps
        sz += r.gyro_z_dps
        n += 1
        time.sleep(0.01)
    if n == 0:
        raise RuntimeError("IMU returned no valid readings during calibration")
    return sx / n, sy / n, sz / n


def main():
    print("ARGOS — 9DOF AHRS test  (Ctrl-C to stop)\n")

    # --- IMU ---
    imu = MPU6050()
    print("  MPU-6050 opened at I2C 0x68")

    # --- Flotilla (optional) ---
    flotilla = None
    if not IMU_ONLY:
        try:
            from argos.sensorium.flotilla import FlotillaReader
            flotilla = FlotillaReader()
            flotilla.start()
            print("  Flotilla dock opened")
            # Give the reader thread a moment to receive initial updates
            time.sleep(1.0)
            m = flotilla.motion_channel(BODY_MOTION_CH)
            chs = flotilla.connected_modules
            if m is None:
                print(f"  WARNING: no Motion on ch {BODY_MOTION_CH} "
                      f"(connected: {chs}) — running 6DOF")
            else:
                print(f"  Body Motion on ch {BODY_MOTION_CH} detected — 9DOF mode")
                print(f"  All channels: {chs}")
        except Exception as exc:
            print(f"  Flotilla not available ({exc}) — running 6DOF")
            flotilla = None

    if IMU_ONLY:
        print("  Running in 6DOF mode (--imu-only)")

    # --- gyro calibration ---
    print(f"\n  Calibrating gyro bias ({CALIB_SECS} s) — keep robot still ...",
          end="", flush=True)
    bias_x, bias_y, bias_z = calibrate_gyro(imu, CALIB_SECS)
    print(" done")
    print(f"  Bias (°/s):  X={bias_x:+.3f}  Y={bias_y:+.3f}  Z={bias_z:+.3f}\n")

    # --- filter ---
    ahrs = MadgwickAHRS(beta=BETA)

    # Seed quaternion from gravity so the filter is near correct orientation
    # immediately instead of converging slowly from identity over ~5 s.
    r0 = imu_read_safe(imu)
    if r0 is not None:
        ahrs.init_from_accel(r0.accel_x_g, r0.accel_y_g, r0.accel_z_g)

    # Short warm-up so the filter is settled before we zero it.
    print("  Levelling (1 s) — keep robot still ...", end="", flush=True)
    t_lev = time.monotonic()
    t_prev_lev = t_lev
    while time.monotonic() - t_lev < 1.0:
        t_now = time.monotonic()
        dt_lev = t_now - t_prev_lev
        t_prev_lev = t_now
        r = imu_read_safe(imu)
        if r is None:
            continue
        gyro_lev = (
            math.radians(r.gyro_x_dps - bias_x),
            math.radians(r.gyro_y_dps - bias_y),
            math.radians(r.gyro_z_dps - bias_z),
        )
        ahrs.update(gyro=gyro_lev,
                    accel=(r.accel_x_g, r.accel_y_g, r.accel_z_g),
                    dt=dt_lev)
        elapsed = time.monotonic() - t_now
        if DT - elapsed > 0:
            time.sleep(DT - elapsed)

    roll_off, pitch_off = ahrs.calibrate_level()
    print(f" done  (mounting offset: roll={roll_off:+.1f}°  pitch={pitch_off:+.1f}°)\n")

    print(f"  {'Roll':>8}  {'Pitch':>8}  {'Yaw':>8}  {'Mode':>6}")
    print(f"  {'(°)':>8}  {'(°)':>8}  {'(°)':>8}")
    print("  " + "-" * 40)

    try:
        t_prev = time.monotonic()
        while True:
            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now

            r = imu_read_safe(imu)
            if r is None:
                continue

            # Gyro in rad/s, bias-corrected
            gyro = (
                math.radians(r.gyro_x_dps - bias_x),
                math.radians(r.gyro_y_dps - bias_y),
                math.radians(r.gyro_z_dps - bias_z),
            )
            accel = (r.accel_x_g, r.accel_y_g, r.accel_z_g)

            # Magnetometer from body-mounted Flotilla Motion (ch 6)
            mag = None
            mode = "6DOF"
            if flotilla is not None:
                m = flotilla.motion_channel(BODY_MOTION_CH)
                if m is not None:
                    mag = (m.mag_x, m.mag_y, m.mag_z)
                    mode = "9DOF"

            ahrs.update(gyro=gyro, accel=accel, mag=mag, dt=dt)

            print(f"\r  {ahrs.roll:>+8.2f}  {ahrs.pitch:>+8.2f}  "
                  f"{ahrs.yaw:>8.2f}  {mode:>6}",
                  end="", flush=True)

            # Pace the loop
            elapsed = time.monotonic() - t_now
            remaining = DT - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\n\nStopped.")
    finally:
        imu.close()
        if flotilla is not None:
            flotilla.close()


if __name__ == "__main__":
    main()
