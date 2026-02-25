"""
Random motor jig for ARGOS.

Fires random moves across all motors in sequence.

Run with: python3 tests/jig.py
          python3 tests/jig.py [steps] [min_speed] [max_speed] [min_dur] [max_dur]

Defaults: 12 steps, speed 55-90%, duration 0.3-0.8s
Speed is always above +/-min_speed (never near zero).
"""

import sys
import pathlib
import random
import subprocess
import argparse

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from argos.config import TRACK_MOTORS, ARM_JOINTS

_MOTORS = (
    [("i2c",  cfg.motor_id) for cfg in TRACK_MOTORS.values()] +
    [("gpio", cfg.motor_id) for cfg in ARM_JOINTS.values()]
)


def rand_speed(lo, hi):
    magnitude = random.randint(lo, hi)
    return magnitude if random.random() > 0.5 else -magnitude


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ARGOS random motor jig")
    parser.add_argument("steps",     nargs="?", type=int,   default=12,  help="number of moves (default 12)")
    parser.add_argument("min_speed", nargs="?", type=int,   default=55,  help="minimum speed %% (default 55)")
    parser.add_argument("max_speed", nargs="?", type=int,   default=90,  help="maximum speed %% (default 90)")
    parser.add_argument("min_dur",   nargs="?", type=float, default=0.3, help="minimum duration s (default 0.3)")
    parser.add_argument("max_dur",   nargs="?", type=float, default=0.8, help="maximum duration s (default 0.8)")
    args = parser.parse_args()

    print(f"ARGOS — jig  ({args.steps} steps, speed {args.min_speed}-{args.max_speed}%, "
          f"duration {args.min_dur}-{args.max_dur}s)\n")

    for step in range(1, args.steps + 1):
        mtype, mid = random.choice(_MOTORS)
        speed    = rand_speed(args.min_speed, args.max_speed)
        duration = round(random.uniform(args.min_dur, args.max_dur), 1)
        print(f"  [{step}/{args.steps}]", end=" ", flush=True)
        subprocess.run(
            [sys.executable, "tests/motor_jog.py", mtype, str(mid), str(speed), str(duration)],
            stdout=subprocess.DEVNULL,
        )
        print(f"{mtype.upper()} motor {mid}  {speed:+d}%  {duration}s")

    print("\ndone")
