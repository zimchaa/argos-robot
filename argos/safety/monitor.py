"""
Safety layer for ARGOS — unconditional authority over all motors.

Wraps TrackedBase and RobotArm with:
  - Speed clamping (max_speed and min_speed per motor from config.py)
  - Watchdog auto-stop (max_duration per motor from config.py)
  - Emergency stop callable that overrides everything above

Usage:
    from argos.safety.monitor import SafetyMonitor

    safety = SafetyMonitor()
    try:
        safety.base.forward(50)         # clamped to max_speed, auto-stops after 10 s
        safety.arm.shoulder.run(60)     # clamped to 80 %, auto-stops after 3 s
        safety.arm.gripper.run(100)     # clamped to 80 %, auto-stops after 2 s
        safety.emergency_stop()         # kills everything immediately
    finally:
        safety.close()
"""

import threading
import time

from argos.base.tracks import TrackedBase
from argos.arm.joints import RobotArm
from argos.config import TRACK_MOTORS, ARM_JOINTS


def _clamp_speed(speed, config):
    """
    Apply max_speed and min_speed limits from a MotorConfig.

    - speed == 0  → 0 (explicit stop always passes through)
    - |speed| > max_speed → clamped down to max_speed
    - |speed| < min_speed → clamped up to min_speed
    Sign is preserved throughout.
    """
    if speed == 0:
        return 0
    sign = 1 if speed > 0 else -1
    magnitude = abs(speed)
    magnitude = min(magnitude, config.max_speed)
    magnitude = max(magnitude, config.min_speed)
    return sign * magnitude


class SafeJoint:
    """
    Wraps a single GPIOMotor for one named arm joint.
    Proxies run() and stop() through the monitor's clamping and watchdog.
    """

    def __init__(self, motor, config, monitor):
        self._motor   = motor
        self._config  = config
        self._monitor = monitor

    def run(self, speed):
        self._monitor._checked_run(self._motor, speed, self._config)

    def stop(self):
        self._monitor._checked_run(self._motor, 0, self._config)


class SafeBase:
    """
    Wraps TrackedBase with speed clamping and watchdog registration.
    Mirrors TrackedBase's full interface (forward/backward/turn_left/
    turn_right/pivot_left/pivot_right/stop/close).
    Drives each track individually so per-motor limits apply correctly.
    """

    def __init__(self, base, monitor):
        self._base      = base
        self._monitor   = monitor
        self._left_cfg  = TRACK_MOTORS["left"]
        self._right_cfg = TRACK_MOTORS["right"]

    def _run_both(self, left_speed, right_speed):
        self._monitor._checked_run(self._base.left,  left_speed,  self._left_cfg)
        self._monitor._checked_run(self._base.right, right_speed, self._right_cfg)

    def forward(self, speed=50):
        self._run_both(speed, speed)

    def backward(self, speed=50):
        self._run_both(-speed, -speed)

    def turn_left(self, speed=50):
        """Gentle arc left: left track half-speed, right full."""
        self._run_both(speed // 2, speed)

    def turn_right(self, speed=50):
        """Gentle arc right: right track half-speed, left full."""
        self._run_both(speed, speed // 2)

    def pivot_left(self, speed=50):
        """Spin in place: tracks opposing."""
        self._run_both(-speed, speed)

    def pivot_right(self, speed=50):
        self._run_both(speed, -speed)

    def stop(self):
        self._run_both(0, 0)

    def close(self):
        self.stop()
        from argos.drivers.pca9685 import I2CMotor
        I2CMotor.close_all()


class SafeArm:
    """
    Wraps RobotArm with speed clamping and watchdog registration.
    Exposes shoulder, elbow, wrist, gripper as SafeJoint instances.
    """

    def __init__(self, arm, monitor):
        self._arm     = arm
        self.shoulder = SafeJoint(arm.shoulder, ARM_JOINTS["shoulder"], monitor)
        self.elbow    = SafeJoint(arm.elbow,    ARM_JOINTS["elbow"],    monitor)
        self.wrist    = SafeJoint(arm.wrist,    ARM_JOINTS["wrist"],    monitor)
        self.gripper  = SafeJoint(arm.gripper,  ARM_JOINTS["gripper"],  monitor)

    def stop(self):
        for joint in (self.shoulder, self.elbow, self.wrist, self.gripper):
            joint.stop()

    def cleanup(self):
        self._arm.cleanup()


class SafetyMonitor:
    """
    Main safety entry point. Owns TrackedBase and RobotArm instances.

    All motion passes through speed clamping and watchdog auto-stop.
    emergency_stop() has unconditional authority: it kills every motor
    regardless of what any higher layer has requested.
    """

    def __init__(self):
        self._base_ctrl = TrackedBase()
        self._arm_ctrl  = RobotArm()
        self.base = SafeBase(self._base_ctrl, self)
        self.arm  = SafeArm(self._arm_ctrl, self)

        self._lock   = threading.Lock()
        self._active = {}   # id(motor) → (deadline: float, motor)

        self._watchdog = threading.Thread(
            target=self._watchdog_loop, daemon=True, name="argos-watchdog"
        )
        self._watchdog.start()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _checked_run(self, motor, speed, config):
        """Clamp speed, drive the motor, update watchdog registration."""
        clamped = _clamp_speed(speed, config)
        motor.run(clamped)
        key = id(motor)
        with self._lock:
            if clamped != 0:
                deadline = time.monotonic() + config.max_duration
                self._active[key] = (deadline, motor)
            else:
                self._active.pop(key, None)

    def _watchdog_loop(self):
        while True:
            time.sleep(0.1)
            now = time.monotonic()
            expired = []
            with self._lock:
                for key, (deadline, motor) in list(self._active.items()):
                    if now >= deadline:
                        expired.append((key, motor))
                for key, _ in expired:
                    del self._active[key]
            for _, motor in expired:
                motor.stop()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def emergency_stop(self):
        """Immediately stop all motors. Overrides any in-progress motion."""
        with self._lock:
            motors = [motor for _, motor in self._active.values()]
            self._active.clear()
        for motor in motors:
            motor.stop()
        # Belt-and-braces: stop subsystems even if not tracked in watchdog dict
        self._base_ctrl.stop()
        self._arm_ctrl.stop()

    def close(self):
        """Emergency stop then release all hardware resources."""
        self.emergency_stop()
        self._arm_ctrl.cleanup()
        from argos.drivers.pca9685 import I2CMotor
        I2CMotor.close_all()
