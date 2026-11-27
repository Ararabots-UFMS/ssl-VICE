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


class RobotStatePredictor:
    """
    Vision says where the robot was at capture; the commands already sent say what it
    has been told since. Rolling the first through the second gives the state at any
    later instant, without the lag a constant-velocity filter has while braking.

      correct(t, x, y):  roll the estimate to the capture instant through the model,
                         then a linear KF update on [x y vx vy] with the position.
      state_at(t):       roll the corrected estimate on through the commands sent.

    Ported from vice-loop-sim/predictor.py, where it was validated in closed loop.
    """

    def __init__(self, actuation_delay, tau, max_accel, meas_sd=15.0, accel_sd=1500.0,
                 step=0.002, history=1.0):
        self.model = RobotModel(actuation_delay, tau, max_accel, step=step, history=history)
        self.R = np.eye(2) * meas_sd ** 2
        self.accel_sd = accel_sd
        self.t = None                  # instant the estimate refers to
        self.x = np.zeros(4)
        self.P = np.diag([meas_sd ** 2] * 2 + [3000.0 ** 2] * 2)
        self._last_stamp = None

    def record_command(self, t_sent, vx, vy):
        """mm/s, field frame: exactly what was published, at the instant it was."""
        self.model.record_command(t_sent, vx, vy)
        self.model.prune(self.t if self.t is not None else t_sent)

    def correct(self, t_capture, x, y):
        if self._last_stamp is not None and t_capture <= self._last_stamp:
            return
        self._last_stamp = t_capture
        z = np.array([x, y], dtype=float)
        if self.t is None:
            self.t, self.x = t_capture, np.array([z[0], z[1], 0.0, 0.0])
            return
        dt = max(0.0, t_capture - self.t)
        self.x = self.model.roll(self.t, self.x, t_capture)
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        Q = G @ G.T * self.accel_sd ** 2
        self.P = F @ self.P @ F.T + Q
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1.0
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - H @ self.x)
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T  # Joseph form
        self.t = t_capture

    @property
    def last_capture(self):
        return self.t

    def state_at(self, t):
        """(x, y, vx, vy) at t, or None before the first correct()."""
        if self.t is None:
            return None
        x = self.x if t <= self.t else self.model.roll(self.t, self.x, t)
        return tuple(float(v) for v in x)
