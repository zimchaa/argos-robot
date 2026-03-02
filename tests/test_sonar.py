"""
HC-SR04 sonar live read test for ARGOS.

Prints distance in cm at ~5 Hz until Ctrl-C.

Usage:
    python3 tests/test_sonar.py

Expected:
  - Flat surface at known distance → reading within ±1 cm.
  - No target / sensor pointing into open space → "  ---" (None).
  - Min reliable range ≈ 2 cm; max ≈ 400 cm.
"""

import sys
import pathlib
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from argos.sensorium.sonar import HCSR04

INTERVAL = 0.2   # seconds between readings (must be ≥ 0.06 s)

if __name__ == "__main__":
    print("ARGOS — HC-SR04 sonar test  (Ctrl-C to stop)\n")

    sonar = HCSR04()
    print("HC-SR04 initialised on BOARD 29 (TRIG) / 31 (ECHO)\n")
    print(f"{'Distance':>12}")
    print(f"{'(cm)':>12}")
    print("-" * 14)

    try:
        while True:
            dist = sonar.read_cm()
            if dist is None:
                print(f"{'---':>12}", flush=True)
            else:
                print(f"{dist:>12.1f}", flush=True)
            time.sleep(INTERVAL)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sonar.close()
