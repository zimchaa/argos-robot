"""
IR proximity sensor live read test for ARGOS.

Prints the state of both sensors at ~10 Hz until Ctrl-C.
Updates in-place so you can watch the indicators change as you
move objects in front of each sensor.

Usage:
    python3 tests/test_ir.py

If a sensor consistently shows DETECTED with nothing in front of it,
trim its potentiometer anticlockwise to reduce sensitivity.
If it never triggers, trim clockwise.

If active_low behaviour seems inverted for your modules, run with:
    ACTIVE_HIGH=1 python3 tests/test_ir.py
"""

import os
import sys
import pathlib
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from argos.sensorium.ir import IRPair

INTERVAL  = 0.1    # seconds between reads
ACTIVE_LOW = os.environ.get("ACTIVE_HIGH") != "1"

_DETECTED = "DETECTED"
_CLEAR    = "clear   "


def _label(state: bool) -> str:
    return _DETECTED if state else _CLEAR


if __name__ == "__main__":
    print("ARGOS — IR proximity sensor test  (Ctrl-C to stop)\n")

    ir = IRPair(active_low=ACTIVE_LOW)
    print(f"Sensors initialised  (active_low={ACTIVE_LOW})")
    print("BOARD 7 = IR1 (CN9)   BOARD 12 = IR2 (CN8)\n")

    try:
        while True:
            s1, s2 = ir.read()
            r1, r2 = ir.ir1.raw(), ir.ir2.raw()
            print(
                f"\r  IR1 (BOARD  7): {_label(s1)}  raw={r1}"
                f"    IR2 (BOARD 12): {_label(s2)}  raw={r2}    ",
                end="",
                flush=True,
            )
            time.sleep(INTERVAL)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ir.close()
