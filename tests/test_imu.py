"""
MPU-6050 IMU live read test for ARGOS.

Prints accelerometer (g), gyroscope (°/s), and die temperature readings
at ~10 Hz until Ctrl-C.  Useful for verifying wiring and orientation.

Usage:
    python3 tests/test_imu.py

Expected when robot is stationary and flat:
  accel Z ≈ +1.0 g  (gravity)
  accel X, Y ≈ 0
  gyro X, Y, Z ≈ 0  (small drift offset is normal — a few °/s)
  temp ≈ ambient + a few °C (die self-heating)
"""

import sys
import pathlib
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from argos.sensorium.imu import MPU6050

INTERVAL = 0.1   # seconds between reads

if __name__ == "__main__":
    print("ARGOS — MPU-6050 IMU test  (Ctrl-C to stop)\n")

    imu = MPU6050()
    print("MPU-6050 initialised OK (WHO_AM_I passed)\n")
    print(f"{'accel X':>10} {'accel Y':>10} {'accel Z':>10}  "
          f"{'gyro X':>10} {'gyro Y':>10} {'gyro Z':>10}  "
          f"{'temp °C':>8}")
    print(f"{'(g)':>10} {'(g)':>10} {'(g)':>10}  "
          f"{'(°/s)':>10} {'(°/s)':>10} {'(°/s)':>10}  "
          f"{'':>8}")
    print("-" * 85)

    try:
        while True:
            r = imu.read()
            print(
                f"{r.accel_x_g:+10.4f} {r.accel_y_g:+10.4f} {r.accel_z_g:+10.4f}  "
                f"{r.gyro_x_dps:+10.3f} {r.gyro_y_dps:+10.3f} {r.gyro_z_dps:+10.3f}  "
                f"{r.temperature_c:8.2f}",
                flush=True,
            )
            time.sleep(INTERVAL)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        imu.close()
