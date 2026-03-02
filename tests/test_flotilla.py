"""tests/test_flotilla.py — live Flotilla sensor dump.

Two modes:

  python3 tests/test_flotilla.py          # parsed readings, updated every 0.5 s
  python3 tests/test_flotilla.py --raw    # raw serial lines (protocol debugging)

The --raw mode is useful for verifying the exact message format from the
dock before trusting the parsed output.
"""

import sys
import time
import glob
import logging

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")


# ---------------------------------------------------------------------------
# --raw mode: just dump serial lines so we can see the protocol
# ---------------------------------------------------------------------------

def dump_raw():
    import serial
    candidates = sorted(glob.glob("/dev/ttyACM*"))
    if not candidates:
        print("No /dev/ttyACM* found — is the dock plugged in?")
        sys.exit(1)
    port = candidates[0]
    print(f"Raw serial dump from {port} at 115200 baud.  Ctrl-C to stop.\n")
    with serial.Serial(port, 115200, timeout=1.0) as s:
        try:
            while True:
                raw = s.readline()
                if raw:
                    print(repr(raw.decode("ascii", errors="replace").rstrip()))
        except KeyboardInterrupt:
            pass


# ---------------------------------------------------------------------------
# Normal mode: parsed readings
# ---------------------------------------------------------------------------

def dump_readings():
    from argos.sensorium.flotilla import FlotillaReader

    reader = FlotillaReader().start()

    print("Connected — waiting 1.5 s for first readings…  Ctrl-C to stop\n")
    time.sleep(1.5)

    mods = reader.connected_modules
    if mods:
        for ch, mod in sorted(mods.items()):
            print(f"  Channel {ch}: {mod}")
    else:
        print("  (no modules reported yet)")
    print()

    try:
        while True:
            m  = reader.motion
            m2 = reader.motion2
            w  = reader.weather
            c  = reader.colour

            if m:
                print(
                    f"Motion1  "
                    f"acc=({m.acc_x:+6d}, {m.acc_y:+6d}, {m.acc_z:+6d})  "
                    f"mag=({m.mag_x:+6d}, {m.mag_y:+6d}, {m.mag_z:+6d})"
                )
            if m2:
                print(
                    f"Motion2  "
                    f"acc=({m2.acc_x:+6d}, {m2.acc_y:+6d}, {m2.acc_z:+6d})  "
                    f"mag=({m2.mag_x:+6d}, {m2.mag_y:+6d}, {m2.mag_z:+6d})"
                )
            if w:
                print(
                    f"Weather  {w.temperature_c:.1f} °C  "
                    f"{w.pressure_hpa:.1f} hPa  "
                    f"(raw: temp={w.temperature_raw}  pres={w.pressure_raw})"
                )
            if c:
                print(
                    f"Colour   "
                    f"R={c.red:5d}  G={c.green:5d}  "
                    f"B={c.blue:5d}  clear={c.clear:5d}"
                )
            if not any([m, m2, w, c]):
                print("(no readings yet — check module connections)")

            print("---")
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        reader.close()


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if "--raw" in sys.argv:
        dump_raw()
    else:
        dump_readings()
