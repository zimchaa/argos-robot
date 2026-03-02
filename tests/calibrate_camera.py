"""
Camera calibration for ARGOS.

Headless-safe — no display required.  Captures frames from the webcam,
detects a checkerboard pattern, and computes the camera intrinsic matrix
and distortion coefficients.  Results are saved to
argos/vision/calibration/camera_matrix.npy and dist_coeffs.npy.

Usage
-----
    python3 tests/calibrate_camera.py [device_index]

Before running
--------------
Print a checkerboard with 9×6 inner corners (10×7 squares, 25 mm each).
An A4 sheet is large enough.  You can generate one with:

    python3 -c "
    import cv2, numpy as np
    board = np.zeros((175, 225), np.uint8)
    sq = 25
    for r in range(7):
        for c in range(10):
            if (r + c) % 2 == 0:
                board[r*sq:(r+1)*sq, c*sq:(c+1)*sq] = 255
    import cv2
    cv2.imwrite('checkerboard_9x6_25mm.png', board)
    print('Saved checkerboard_9x6_25mm.png')
    "

Steps
-----
1. Run this script — it captures a frame every 2 seconds.
2. Hold the printed board in front of the camera.
3. After each 'ACCEPTED' message, move the board to a new position:
     - tilt toward each corner of the frame
     - vary the distance (30–60 cm works well at 640×480)
     - tilt left/right and up/down
4. After 20 good frames the calibration runs automatically.
5. Results are saved to argos/vision/calibration/.

Debug frames (with corners drawn) are saved to tests/calibration_frames/
so you can review what was captured.

Quality guide
-------------
RMS reprojection error < 0.5 px  — excellent
                       0.5–1.0   — acceptable
                       > 1.0     — recapture with a flatter board and more angle variety
"""

import sys
import pathlib
import time
import json

import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from argos.vision.camera import Camera

# ---------------------------------------------------------------------------
# Checkerboard parameters — must match the printed board
# ---------------------------------------------------------------------------
BOARD_W        = 9       # inner corners wide
BOARD_H        = 6       # inner corners tall
SQUARE_SIZE_MM = 25.0    # physical square size in mm

# ---------------------------------------------------------------------------
# Capture parameters
# ---------------------------------------------------------------------------
TARGET_FRAMES    = 20    # stop collecting after this many accepted frames
CAPTURE_INTERVAL = 2.0   # seconds between capture attempts

# Minimum movement of board centroid (as fraction of image diagonal)
# to accept a new frame — prevents near-duplicate captures
MIN_MOVE_FRAC = 0.08

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
REPO_ROOT  = pathlib.Path(__file__).resolve().parents[1]
OUT_DIR    = REPO_ROOT / "argos" / "vision" / "calibration"
DEBUG_DIR  = pathlib.Path(__file__).parent / "calibration_frames"

BOARD_SIZE = (BOARD_W, BOARD_H)

SUBPIX_CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001
)


def _object_points():
    """3D object points for one board pose in the Z=0 plane."""
    obj = np.zeros((BOARD_H * BOARD_W, 3), np.float32)
    obj[:, :2] = np.mgrid[0:BOARD_W, 0:BOARD_H].T.reshape(-1, 2)
    obj *= SQUARE_SIZE_MM
    return obj


def _centroid(corners):
    return corners[:, 0, :].mean(axis=0)


def _is_new_enough(corners, accepted, img_diagonal):
    """True if this board position is far enough from all previously accepted frames."""
    if not accepted:
        return True
    threshold = MIN_MOVE_FRAC * img_diagonal
    c = _centroid(corners)
    for prev in accepted:
        if np.linalg.norm(c - _centroid(prev)) < threshold:
            return False
    return True


def main():
    device = int(sys.argv[1]) if len(sys.argv) > 1 else 0

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)

    obj_pt = _object_points()

    obj_points       = []   # accumulated 3D points, one entry per accepted frame
    img_points       = []   # accumulated 2D corner points
    accepted_corners = []   # for diversity check

    print("ARGOS — camera calibration")
    print(f"  Board    : {BOARD_W}×{BOARD_H} inner corners, {SQUARE_SIZE_MM:.0f} mm squares")
    print(f"  Target   : {TARGET_FRAMES} good frames")
    print(f"  Interval : {CAPTURE_INTERVAL:.0f} s between attempts")
    print(f"  Output   : {OUT_DIR}")
    print(f"  Debug    : {DEBUG_DIR}")
    print()
    print("Hold the checkerboard in front of the camera.")
    print("Move it to a new position/angle after each ACCEPTED frame.")
    print("Cover all corners of the frame and a range of tilt angles.")
    print()

    with Camera(index=device) as cam:
        w, h = cam.actual_resolution()
        img_diagonal = (w ** 2 + h ** 2) ** 0.5
        print(f"  Camera   : {w}×{h} @ {cam.actual_fps():.0f} fps\n")

        # Warm up — some cameras return dark frames while auto-exposure settles
        for _ in range(5):
            cam.capture()
            time.sleep(0.05)

        attempt = 0
        while len(accepted_corners) < TARGET_FRAMES:
            frame = cam.capture()
            attempt += 1
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            found, corners = cv2.findChessboardCorners(
                gray, BOARD_SIZE,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )

            if not found:
                print(f"  [{attempt:3d}]  board not detected", flush=True)
                time.sleep(CAPTURE_INTERVAL)
                continue

            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA
            )

            if not _is_new_enough(corners_refined, accepted_corners, img_diagonal):
                print(
                    f"  [{attempt:3d}]  detected — too similar to a previous frame"
                    "  (move the board)",
                    flush=True,
                )
                time.sleep(CAPTURE_INTERVAL)
                continue

            # Accept
            n = len(accepted_corners) + 1
            accepted_corners.append(corners_refined)
            obj_points.append(obj_pt)
            img_points.append(corners_refined)

            debug = cv2.drawChessboardCorners(
                frame.copy(), BOARD_SIZE, corners_refined, True
            )
            debug_path = DEBUG_DIR / f"frame_{n:02d}.jpg"
            cv2.imwrite(str(debug_path), debug)

            print(
                f"  [{attempt:3d}]  ACCEPTED {n:2d}/{TARGET_FRAMES}"
                f"  → {debug_path.name}",
                flush=True,
            )
            print("\a", end="", flush=True)  # terminal bell

            if len(accepted_corners) < TARGET_FRAMES:
                time.sleep(CAPTURE_INTERVAL)

    # ---------------------------------------------------------------------------
    # Calibrate
    # ---------------------------------------------------------------------------
    print(f"\nRunning calibrateCamera on {len(obj_points)} frames ...")

    rms, camera_matrix, dist_coeffs, _rvecs, _tvecs = cv2.calibrateCamera(
        obj_points, img_points, (w, h), None, None
    )

    print(f"  RMS reprojection error : {rms:.4f} px", end="")
    if rms < 0.5:
        print("  (excellent)")
    elif rms < 1.0:
        print("  (acceptable)")
    else:
        print(
            "\n  WARNING: RMS > 1.0 px."
            " Consider recapturing with a flatter board and more angle variety."
        )

    # ---------------------------------------------------------------------------
    # Save
    # ---------------------------------------------------------------------------
    np.save(str(OUT_DIR / "camera_matrix.npy"), camera_matrix)
    np.save(str(OUT_DIR / "dist_coeffs.npy"),   dist_coeffs)

    summary = {
        "rms_px":              float(rms),
        "image_size":          [w, h],
        "board_inner_corners": [BOARD_W, BOARD_H],
        "square_size_mm":      SQUARE_SIZE_MM,
        "frames_used":         len(obj_points),
        "fx":      float(camera_matrix[0, 0]),
        "fy":      float(camera_matrix[1, 1]),
        "cx":      float(camera_matrix[0, 2]),
        "cy":      float(camera_matrix[1, 2]),
        "dist_k1": float(dist_coeffs[0, 0]),
        "dist_k2": float(dist_coeffs[0, 1]),
        "dist_p1": float(dist_coeffs[0, 2]),
        "dist_p2": float(dist_coeffs[0, 3]),
        "dist_k3": float(dist_coeffs[0, 4]),
    }

    with open(OUT_DIR / "calibration_summary.json", "w") as fh:
        json.dump(summary, fh, indent=2)

    print(f"\n  fx = {summary['fx']:.1f}  fy = {summary['fy']:.1f}")
    print(f"  cx = {summary['cx']:.1f}  cy = {summary['cy']:.1f}")
    print()
    print(f"  Saved → {OUT_DIR / 'camera_matrix.npy'}")
    print(f"  Saved → {OUT_DIR / 'dist_coeffs.npy'}")
    print(f"  Saved → {OUT_DIR / 'calibration_summary.json'}")
    print()
    print("Calibration complete.")


if __name__ == "__main__":
    main()
