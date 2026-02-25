"""
Manual hardware test for all motors with interactive reporting.

Run with: python3 tests/test_all_motors_manual.py

- Prompts for default speeds at startup
- Runs each motor individually with a keypress gate
- Collects observations after each run (direction, result, notes)
- Prints a formatted report at the end to paste back into the dev session

NO governor fitted on arm — arm speed is capped at 70%.
Be ready to Ctrl-C at any time.
"""

import sys
import pathlib
import time
import datetime

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import smbus2
import RPi.GPIO as GPIO
from argos.drivers.pca9685 import I2CMotor
from argos.drivers.gpio_motor import GPIOMotor
from argos.base.tracks import TrackedBase
from argos.config import TRACK_MOTORS, ARM_JOINTS

ARM_SPEED_CAP = 70   # hard cap — no governor fitted


def ask_int(prompt, default, lo, hi):
    while True:
        raw = input(f"  {prompt} [{default}]: ").strip()
        if raw == "":
            return default
        try:
            val = int(raw)
            if lo <= val <= hi:
                return val
            print(f"  Enter a value between {lo} and {hi}.")
        except ValueError:
            print("  Enter a whole number.")


def observe(label):
    """Ask for direction observation and optional notes. Returns a dict."""
    print(f"  Observation for {label}:")
    dir_input = input("    Direction for +ve speed (f=forward/up/open, r=reverse/down/close, n=no movement, ?=unclear): ").strip().lower()
    direction_map = {"f": "forward/up/open", "r": "reverse/down/close", "n": "no movement", "?": "unclear"}
    direction = direction_map.get(dir_input, dir_input or "not recorded")
    notes = input("    Any notes (or Enter to skip): ").strip()
    return {"direction": direction, "notes": notes}


def check_comms():
    print("\n-- COMMS CHECK --")
    bus = smbus2.SMBus(1)
    val = bus.read_byte_data(0x40, 0x00)
    bus.close()
    print(f"  PCA9685 at 0x40 — MODE1 = 0x{val:02X}  ✓")
    return f"0x{val:02X}"


def run_i2c_motor(motor_id, label, speed, duration):
    print(f"\n  Running Motor {motor_id} ({label}) at {speed}% for {duration}s ...")
    m = I2CMotor(motor_id=motor_id)
    m.run(speed)
    time.sleep(duration)
    m.stop()
    I2CMotor.close_all()
    print(f"  Stopped.")


def run_base(speed, duration):
    print(f"\n  Running TrackedBase.forward({speed}%) for {duration}s ...")
    base = TrackedBase()
    base.forward(speed)
    time.sleep(duration)
    base.stop()
    base.close()
    print(f"  Stopped.")


def run_joint(motor_id, label, speed, duration):
    print(f"\n  Running joint {motor_id} ({label}) at +{speed}% for {duration}s ...")
    m = GPIOMotor(motor_id)
    m.run(speed)
    time.sleep(duration)
    m.cleanup()
    print(f"  Stopped.")


def print_report(report):
    lines = []
    lines.append("=" * 60)
    lines.append("ARGOS MOTOR TEST REPORT")
    lines.append(f"Date: {report['date']}")
    lines.append(f"PCA9685 MODE1: {report['mode1']}")
    lines.append("")
    lines.append("TRACKS (Waveshare HAT, I2C 0x40)")
    lines.append(f"  Speed: {report.get('track_speed', '?')}%   Duration: {report.get('track_duration', '?')}s")
    for entry in report["tracks"]:
        lines.append(f"  Motor {entry['id']} ({entry['label']}): {entry['direction']}")
        if entry["notes"]:
            lines.append(f"    Notes: {entry['notes']}")
    lines.append("")
    lines.append("TRACKS COMBINED (TrackedBase.forward)")
    entry = report.get("base", {})
    lines.append(f"  Result: {entry.get('direction', 'not reached')}")
    if entry.get("notes"):
        lines.append(f"  Notes: {entry['notes']}")
    lines.append("")
    lines.append("ARM JOINTS (SB Components MotorShield, GPIO)")
    lines.append(f"  Speed: {report.get('arm_speed', '?')}%   Duration: {report.get('arm_duration', '?')}s")
    for entry in report["joints"]:
        lines.append(f"  Motor {entry['id']} ({entry['label']}): {entry['direction']}")
        if entry["notes"]:
            lines.append(f"    Notes: {entry['notes']}")
    lines.append("=" * 60)
    output = "\n".join(lines)
    print("\n\n" + output + "\n")


if __name__ == "__main__":
    report = {
        "date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
        "mode1": "—",
        "tracks": [],
        "base": {},
        "joints": [],
    }

    try:
        # --- Startup config ---
        print("=" * 60)
        print("ARGOS — full motor test")
        print("=" * 60)
        print("\nSet default speeds (press Enter to accept defaults):")
        track_speed    = ask_int("Track speed % (1-100)", 70, 1, 100)
        track_duration = ask_int("Track run duration seconds (1-5)", 1, 1, 5)
        arm_speed_raw  = ask_int(f"Arm speed % (1-{ARM_SPEED_CAP}, no governor fitted)", 25, 1, ARM_SPEED_CAP)
        arm_speed      = min(arm_speed_raw, ARM_SPEED_CAP)
        arm_duration   = ask_int("Arm run duration seconds (1-3)", 1, 1, 3)

        report["track_speed"]    = track_speed
        report["track_duration"] = track_duration
        report["arm_speed"]      = arm_speed
        report["arm_duration"]   = arm_duration

        # --- Comms ---
        report["mode1"] = check_comms()

        # --- Tracks ---
        print("\n-- TRACKS --")
        for label, cfg in TRACK_MOTORS.items():
            motor_id = cfg.motor_id
            input(f"\n  Press Enter to run Motor {motor_id} ({label}) ...")
            run_i2c_motor(motor_id, label, track_speed, track_duration)
            obs = observe(label)
            report["tracks"].append({"id": motor_id, "label": label, **obs})

        input("\n  Press Enter to run both tracks together (TrackedBase.forward) ...")
        run_base(track_speed, track_duration)
        obs = observe("both tracks")
        report["base"] = obs

        # --- Arm joints ---
        print(f"\n-- ARM JOINTS (speed capped at {ARM_SPEED_CAP}%, no governor) --")
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        for label, cfg in ARM_JOINTS.items():
            motor_id = cfg.motor_id
            input(f"\n  Press Enter to jog {label} ...")
            run_joint(motor_id, label, arm_speed, arm_duration)
            obs = observe(label)
            report["joints"].append({"id": motor_id, "label": label, **obs})
        GPIO.cleanup()

        print_report(report)

    except KeyboardInterrupt:
        print("\n\nAborted — cleaning up ...")
        try:
            I2CMotor.close_all()
        except Exception:
            pass
        try:
            GPIO.cleanup()
        except Exception:
            pass
        if report["tracks"] or report["joints"]:
            report.setdefault("base", {"direction": "aborted", "notes": ""})
            print_report(report)
    except Exception as e:
        print(f"\nERROR: {e}")
        try:
            GPIO.cleanup()
        except Exception:
            pass
        raise
