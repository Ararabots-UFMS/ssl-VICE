"""
Where a robot will be when the command computed now takes effect.

Units: mm, mm/s, mm/s², seconds on the ROS clock. No ROS imports, so the
identification fit and the tests use it directly.
"""
from collections import deque

import numpy as np


class RobotModel:
    """
    Command -> motion. A command published at t_sent acts from t_sent + actuation_delay
    and holds until the next one. Velocity follows it first-order (tau), the change per
    step capped at max_accel * dt (vector norm); position integrates velocity with the
    trapezoid rule.

    tau and max_accel may be arrays of shape (N, 1) to roll N parameter sets at once
    (the identification fit does that); states are then (N, 4).
    """

    def __init__(self, actuation_delay, tau, max_accel, step=0.002, history=1.0):
        self.actuation_delay = float(actuation_delay)
        self.tau = tau
        self.max_accel = max_accel
        self.step = step
        self.history = history
        self._commands = deque()  # (t_effective, [vx, vy]), in publish order

    def record_command(self, t_sent, vx, vy):
        self._commands.append((t_sent + self.actuation_delay, np.array([vx, vy], dtype=float)))

    def prune(self, keep_from):
        """Drop commands superseded before keep_from - history; the one in effect stays."""
        if self.history is None:
            return
        horizon = keep_from - self.history
        while len(self._commands) > 2 and self._commands[1][0] < horizon:
            self._commands.popleft()

    def roll(self, t0, state, t1):
        """Integrate state [x, y, vx, vy] from t0 to t1 through the recorded commands."""
        x = np.array(state, dtype=float)
        commands = self._commands
        # Commands are time-ordered, so the one in effect only ever moves forward.
        i, n = 0, len(commands)
        u = np.zeros(2)
        t = t0
        while t < t1 - 1e-12:
            while i < n and commands[i][0] <= t:
                u = commands[i][1]
                i += 1
            dt = min(self.step, t1 - t)
            v = x[..., 2:]
            dv = (u - v) * (dt / self.tau)
            norm = np.linalg.norm(dv, axis=-1, keepdims=True)
            limit = self.max_accel * dt
            dv = dv * np.minimum(1.0, limit / np.maximum(norm, 1e-12))
            v_new = v + dv
            x[..., :2] += 0.5 * (v + v_new) * dt
            x[..., 2:] = v_new
            t += dt
        return x
