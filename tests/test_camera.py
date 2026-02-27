"""
USB webcam test for ARGOS.

Captures a single frame, saves it to tests/camera_test.jpg, and reports
the actual resolution and FPS the driver negotiated.

Headless-safe — no display or GUI required.

Usage:
    python3 tests/test_camera.py [index]

    index  V4L2 device index (default 0). Try 1 if 0 fails.
           Run 'ls /dev/video*' to see available devices.

Install opencv if needed:
    pip3 install opencv-python-headless
  or:
    sudo apt install python3-opencv
"""

import sys
import pathlib
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from argos.vision.camera import Camera

OUTPUT = pathlib.Path(__file__).parent / "camera_test.jpg"

index = int(sys.argv[1]) if len(sys.argv) > 1 else 0

print(f"ARGOS — camera test  (device index {index})\n")

try:
    with Camera(index=index) as cam:
        w, h = cam.actual_resolution()
        fps  = cam.actual_fps()
        print(f"  Opened /dev/video{index}")
        print(f"  Resolution : {w} × {h}")
        print(f"  FPS        : {fps}")

        # Discard a few frames — some cameras return a dark or blank
        # first frame while auto-exposure settles.
        print("\n  Warming up (5 frames) ...", end="", flush=True)
        for _ in range(5):
            cam.capture()
            time.sleep(0.05)
        print(" done")

        frame = cam.capture()
        print(f"  Frame shape: {frame.shape}  dtype={frame.dtype}")

        import cv2
        cv2.imwrite(str(OUTPUT), frame)
        print(f"\n  Saved → {OUTPUT}")
        print("\nTest passed.")

except RuntimeError as exc:
    print(f"\nERROR: {exc}", file=sys.stderr)
    sys.exit(1)
except ImportError:
    print(
        "\nERROR: opencv not installed.\n"
        "  pip3 install opencv-python-headless\n"
        "or:\n"
        "  sudo apt install python3-opencv",
        file=sys.stderr,
    )
    sys.exit(1)
