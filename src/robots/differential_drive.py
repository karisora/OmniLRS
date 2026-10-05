"""Differential wheel kinematics and command watchdog, independent of Isaac/ROS."""

import math
from threading import Lock
import time


class DifferentialDrive:
    def __init__(self, wheel_radius, wheel_track, max_wheel_speed=20.0, command_timeout=0.5, topic="/cmd_vel"):
        for name, value in [
            ("wheel_radius", wheel_radius),
            ("wheel_track", wheel_track),
            ("max_wheel_speed", max_wheel_speed),
            ("command_timeout", command_timeout),
        ]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive.")
        self.wheel_radius = wheel_radius
        self.wheel_track = wheel_track
        self.max_wheel_speed = max_wheel_speed
        self.command_timeout = command_timeout
        self.topic = topic
        self._lock = Lock()
        self.reset()

    def reset(self):
        with self._lock:
            self._command = (0.0, 0.0, -math.inf)

    def receive(self, linear, angular, now=None):
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.reset()
            return
        with self._lock:
            self._command = (linear, angular, time.monotonic() if now is None else now)

    def wheel_speeds(self, linear, angular):
        left = (linear - angular * self.wheel_track / 2) / self.wheel_radius
        right = (linear + angular * self.wheel_track / 2) / self.wheel_radius
        # Limit both together so the commanded curvature is preserved.
        scale = max(1.0, abs(left) / self.max_wheel_speed, abs(right) / self.max_wheel_speed)
        return left / scale, right / scale

    def targets(self, wheel_joints, now=None):
        with self._lock:
            linear, angular, stamp = self._command
        now = time.monotonic() if now is None else now
        if now - stamp >= self.command_timeout:
            linear, angular = 0.0, 0.0
        left, right = self.wheel_speeds(linear, angular)
        return {
            joint: speed for side, speed in [("left", left), ("right", right)] for joint in wheel_joints.get(side, [])
        }
