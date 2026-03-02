"""
Camera intrinsics loader.

Run ``python3 tests/calibrate_camera.py`` on hardware to generate the
calibration data files, then import this module to use them.

Usage::

    from argos.vision.calibration import load

    camera_matrix, dist_coeffs = load()
"""

import pathlib
import numpy as np

_DIR = pathlib.Path(__file__).parent


def load():
    """
    Load camera calibration data.

    Returns
    -------
    camera_matrix : numpy.ndarray, shape (3, 3)
        Intrinsic matrix K.
    dist_coeffs : numpy.ndarray, shape (1, 5)
        Distortion coefficients (k1, k2, p1, p2, k3).

    Raises
    ------
    FileNotFoundError
        If calibration data is missing. Run calibrate_camera.py first::

            python3 tests/calibrate_camera.py
    """
    matrix_path = _DIR / "camera_matrix.npy"
    dist_path   = _DIR / "dist_coeffs.npy"

    if not matrix_path.exists() or not dist_path.exists():
        raise FileNotFoundError(
            "Camera calibration data not found.\n"
            f"  Expected : {matrix_path}\n"
            f"             {dist_path}\n"
            "  Run     : python3 tests/calibrate_camera.py"
        )

    return np.load(str(matrix_path)), np.load(str(dist_path))
