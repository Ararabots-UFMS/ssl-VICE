#!/usr/bin/env python

"""
Trapezoidal Steering for a Vector of Double-Integrators

Adapted from "Bang-bang boosting of RRTs", A. J. LaValle, B. Sakcak, and S. M. LaValle.
IEEE/RSJ International Conference on Intelligent Robots and Systems, Oct 2023.

Changes from the original bang-bang version:
  - Added per-axis velocity limits (vmin / vmax) to all planning functions.
  - bang_bang_optimal is now used internally only; the public planning API is
    trapezoidal_optimal / trapezoidal_scaled / trapezoidal_hard_stop /
    trapezoidal_hard_stop_wait.
  - time_optimal_steer and time_optimal_steer_2d now accept vmin / vmax vectors
    and dispatch to the trapezoidal planners.
  - All integrators (integrate_control_2d, integrate_control_2d_timestep,
    integrate_control_2d_at_time, integrate_control_nd) and merge utilities
    are unchanged — the control-sequence format [[u_vec, dt], ...] is identical.

BSD 2-Clause License

Copyright (c) 2023, Alexander J. LaValle
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

from math import sqrt, fabs
import random

time_epsilon = 0.0000001
float_epsilon = 1.0e-200


# ══════════════════════════════════════════════════════════════════════════════
#  HIGH-LEVEL STEERING
# ══════════════════════════════════════════════════════════════════════════════

def time_optimal_steer(xinit, xgoal, umin=0, umax=0, vmin=0, vmax=0):
    """
    Time-optimal trapezoidal steering for a vector of n double integrators.

    State format: 2n-dimensional vector [p0..pn-1, v0..vn-1].
    umin, umax : n-dimensional acceleration bound vectors.
    vmin, vmax : n-dimensional velocity bound vectors.

    Returns a merged control sequence [[u_vec, dt], ...] identical in format
    to the original bang-bang version so all downstream code is unaffected.
    """
    n = int(0.5 * len(xinit))
    if umin == 0:
        umin = [-1.0] * n
    if umax == 0:
        umax = [1.0] * n
    if vmin == 0:
        vmin = [-1.0] * n
    if vmax == 0:
        vmax = [1.0] * n

    cvec = []
    tvec = []
    tmax = 0.0
    imax = 0

    for i in range(n):
        cvec.append(
            trapezoidal_optimal(
                xinit[i], xinit[n + i], xgoal[i], xgoal[n + i],
                umin[i], umax[i], vmin[i], vmax[i],
            )
        )
        tvec.append(control_time(cvec[i]))
        if tvec[i] > tmax:
            tmax = tvec[i]
            imax = i

    i = 0
    restart = False

    while i < n:
        if i != imax and fabs(tvec[i] - tmax) > time_epsilon:
            c = trapezoidal_scaled(
                xinit[i], xinit[n + i], xgoal[i], xgoal[n + i], tmax,
                umin[i], umax[i], vmin[i], vmax[i],
            )
            if c == []:
                c = trapezoidal_hard_stop_wait(
                    xinit[i], xinit[n + i], xgoal[i], xgoal[n + i], tmax,
                    umin[i], umax[i], vmin[i], vmax[i],
                )
            if c == []:
                # Axis cannot stretch to tmax — recompute as new bottleneck
                c = trapezoidal_hard_stop(
                    xinit[i], xinit[n + i], xgoal[i], xgoal[n + i],
                    umin[i], umax[i], vmin[i], vmax[i],
                )
                tmax = control_time(c)
                imax = i
                restart = True
            cvec[i] = c
            tvec[i] = control_time(c)
            if restart:
                restart = False
                i = -1
        i += 1

    return merge_scalar_controls(cvec)


def time_optimal_steer_2d(xinit, xgoal,
                          umin=(-1, -1), umax=(1, 1),
                          vmin=(-1, -1), vmax=(1, 1)):
    """
    Faster specialised version of time_optimal_steer for two double integrators.

    State format: (x, y, xdot, ydot).
    """
    c1 = trapezoidal_optimal(xinit[0], xinit[2], xgoal[0], xgoal[2],
                             umin[0], umax[0], vmin[0], vmax[0])
    c2 = trapezoidal_optimal(xinit[1], xinit[3], xgoal[1], xgoal[3],
                             umin[1], umax[1], vmin[1], vmax[1])

    # Continuously loop until the times of both axes match
    while True:
        t1 = control_time(c1)
        t2 = control_time(c2)

        # If the durations are synchronized, break the loop and merge
        if fabs(t1 - t2) <= time_epsilon:
            break

        # Stretch the faster axis to match the slower one
        if t1 < t2:
            c1 = _stretch_or_hardstop(
                xinit[0], xinit[2], xgoal[0], xgoal[2], t2,
                umin[0], umax[0], vmin[0], vmax[0], c1,
            )
            # Note: If this fails to meet t2 exactly, _stretch_or_hardstop will 
            # fall back to a hard stop that takes longer than t2. On the next loop 
            # iteration, t1 > t2, so the code will seamlessly try to stretch c2.
        else:
            c2 = _stretch_or_hardstop(
                xinit[1], xinit[3], xgoal[1], xgoal[3], t1,
                umin[1], umax[1], vmin[1], vmax[1], c2,
            )

    return merge_scalar_controls([c1, c2])

def _stretch_or_hardstop(ix, iv, gx, gv, tf,
                         umin, umax, vmin, vmax, c_opt):
    """Helper: try trapezoidal_scaled, fall back to trapezoidal_hard_stop_wait."""
    c = trapezoidal_scaled(ix, iv, gx, gv, tf, umin, umax, vmin, vmax)
    if c == []:
        c = trapezoidal_hard_stop_wait(ix, iv, gx, gv, tf, umin, umax, vmin, vmax)
    if c == []:
        # Cannot meet tf — return hard-stop and let caller re-time
        c = trapezoidal_hard_stop(ix, iv, gx, gv, umin, umax, vmin, vmax)
    return c


# ══════════════════════════════════════════════════════════════════════════════
#  CORE 1-D TRAPEZOIDAL PLANNERS
# ══════════════════════════════════════════════════════════════════════════════

def trapezoidal_optimal(ix, iv, gx, gv,
                        umin=-1.0, umax=1.0,
                        vmin=-1.0, vmax=1.0):
    """
    Time-optimal 1-D trapezoidal profile with velocity limits.

    Internally calls bang_bang_optimal. If the bang-bang peak velocity stays
    within [vmin, vmax] the bang-bang solution IS the trapezoidal solution
    (triangular profile). Otherwise a cruise phase at the velocity limit is
    inserted, producing the classic three-phase trapezoid.

    Returns a list of [u, dt] segments (same format as bang_bang_optimal).
    A cruise phase has u = 0.0.
    """
    if ix == gx and iv == gv:
        return []

    # Get the unconstrained time-optimal solution first
    bb = bang_bang_optimal(ix, iv, gx, gv, umin, umax)
    if not bb:
        return []

    # Determine the switching velocity
    v_peak = iv + bb[0][0] * bb[0][1]

    # If peak is within velocity limits, bang-bang is already optimal
    if vmin - time_epsilon <= v_peak <= vmax + time_epsilon:
        return bb

    # ── cruise phase ──────────────────────────────────────────────────
    # Choose cruise velocity and the two acceleration signs
    if v_peak > vmax:
        v_cruise = vmax
        u1 = umax
        u3 = umin
    else:           # v_peak < vmin
        v_cruise = vmin
        u1 = umin   
        u3 = umax 

    # Phase 1: iv → v_cruise
    t1 = (v_cruise - iv) / u1 if fabs(u1) > float_epsilon else 0.0
    d1 = iv * t1 + 0.5 * u1 * t1 ** 2

    # Phase 3: v_cruise → gv
    t3 = (gv - v_cruise) / u3 if fabs(u3) > float_epsilon else 0.0
    d3 = v_cruise * t3 + 0.5 * u3 * t3 ** 2

    # Phase 2: cruise
    d2 = (gx - ix) - d1 - d3
    t2 = d2 / v_cruise if fabs(v_cruise) > float_epsilon else 0.0

    if t1 < -time_epsilon or t3 < -time_epsilon or t2 < -time_epsilon:
        return bb

    t1 = max(0.0, t1)
    t2 = max(0.0, t2)
    t3 = max(0.0, t3)

    result = []
    if t1 > time_epsilon:
        result.append([u1, t1])
    if t2 > time_epsilon:
        result.append([0.0, t2])
    if t3 > time_epsilon:
        result.append([u3, t3])

    return result if result else bb


def trapezoidal_scaled(ix, iv, gx, gv, tf,
                       umin=-1.0, umax=1.0,
                       vmin=-1.0, vmax=1.0):
    """
    Find a control that steers (ix, iv) → (gx, gv) in exactly tf while
    respecting acceleration and velocity limits.

    Strategy:
      1. If already at optimal time, return the optimal profile.
      2. Try bang_bang_scaled — algebraically correct for duration and
         position. Accept only if the peak velocity stays within [vmin, vmax].
         (For triangular profiles this almost always works; for profiles that
         needed a velocity cap it may still work at the longer duration.)
      3. Fall back to trapezoidal_hard_stop_wait (brake → wait → go). Always
         position-correct because the wait happens at zero velocity.

    NOTE: Naive "insert a cruise phase" approaches are intentionally avoided
    because adding travel time at non-zero velocity always shifts the terminal
    position by v_cruise × extra_time.

    Returns [] if tf is less than the minimum achievable time.
    """
    co = trapezoidal_optimal(ix, iv, gx, gv, umin, umax, vmin, vmax)
    tfo = control_time(co)

    if tfo > tf + time_epsilon:
        print("trapezoidal_scaled: requested time is less than time-optimal")
        return []

    if fabs(tfo - tf) <= time_epsilon:
        return co

    # ── Try bang_bang_scaled: position-correct by construction ───────────────
    bb = bang_bang_scaled(ix, iv, gx, gv, tf, umin, umax)
    if bb:
        # Verify the intermediate (peak) velocity is within limits.
        # For 2-phase controls the velocity is monotone within each phase, so
        # checking only the switching point is sufficient.
        v_peak = iv + bb[0][0] * bb[0][1]
        if vmin - time_epsilon <= v_peak <= vmax + time_epsilon:
            return bb

    # ── Fallback: brake to zero, wait, then drive to goal ───────────────────
    return trapezoidal_hard_stop_wait(ix, iv, gx, gv, tf, umin, umax, vmin, vmax)


def trapezoidal_hard_stop(ix, iv, gx, gv,
                          umin=-1.0, umax=1.0,
                          vmin=-1.0, vmax=1.0):
    """
    Brake the current velocity to zero first, then run trapezoidal_optimal
    to the goal. Equivalent to bang_bang_hard_stop but uses the velocity-aware
    planner for the second leg.
    """
    if fabs(iv) < time_epsilon:
        return trapezoidal_optimal(ix, 0.0, gx, gv, umin, umax, vmin, vmax)

    # Choose braking acceleration opposite to current velocity
    ux = umin if iv > 0 else umax
    s = -iv / ux
    sx = ix + iv * s + 0.5 * ux * s ** 2

    d = trapezoidal_optimal(sx, 0.0, gx, gv, umin, umax, vmin, vmax)

    c = []
    if s > time_epsilon:
        c.append([ux, s])
    c += d
    return c


def trapezoidal_hard_stop_wait(ix, iv, gx, gv, tf,
                               umin=-1.0, umax=1.0,
                               vmin=-1.0, vmax=1.0):
    """
    Brake to zero, insert a zero-acceleration wait if needed, then
    run trapezoidal_optimal. Guarantees total duration == tf.

    Returns [] if tf is shorter than the hard-stop + move time.
    """
    c = trapezoidal_hard_stop(ix, iv, gx, gv, umin, umax, vmin, vmax)
    tt = control_time(c)

    if tt > tf + time_epsilon:
        return []

    if tf > tt + time_epsilon:
        wait_dt = tf - tt
        if fabs(iv) < time_epsilon:
            c.insert(0, [0.0, wait_dt])
        else:
            c.insert(1, [0.0, wait_dt])

    return c


# ══════════════════════════════════════════════════════════════════════════════
#  UTILITY  —  control time
# ══════════════════════════════════════════════════════════════════════════════

def control_time(con):
    """Total duration of a control sequence."""
    t = 0.0
    for c in con:
        t += c[1]
    return t


# ══════════════════════════════════════════════════════════════════════════════
#  INTEGRATORS
# ══════════════════════════════════════════════════════════════════════════════

def integrate_control_2d(xi, con):
    """Integrate a 2-D control sequence to its terminal state."""
    x, y, xdot, ydot = xi[0], xi[1], xi[2], xi[3]
    for c in con:
        x    += xdot * c[1] + 0.5 * c[0][0] * c[1] ** 2
        xdot += c[0][0] * c[1]
        y    += ydot * c[1] + 0.5 * c[0][1] * c[1] ** 2
        ydot += c[0][1] * c[1]
    return (x, y, xdot, ydot)


def integrate_control_2d_timestep(xi, con, timestep):
    """
    Integrate the control sequence in fixed timesteps.

    Args:
        xi:        Initial state (x, y, xdot, ydot).
        con:       Control sequence [[accel_vec, duration], ...].
        timestep:  Fixed timestep for integration.

    Returns:
        List of states (x, y, xdot, ydot) at each timestep.
    """
    x, y, xdot, ydot = xi
    states = [(x, y, xdot, ydot)]
    time_elapsed = 0.0

    for c in con:
        ax, ay = c[0]
        duration = c[1]
        while time_elapsed + timestep <= duration:
            x    += xdot * timestep + 0.5 * ax * timestep ** 2
            xdot += ax * timestep
            y    += ydot * timestep + 0.5 * ay * timestep ** 2
            ydot += ay * timestep
            time_elapsed += timestep
            states.append((x, y, xdot, ydot))

        remaining_time = duration - time_elapsed
        if remaining_time > 0:
            x    += xdot * remaining_time + 0.5 * ax * remaining_time ** 2
            xdot += ax * remaining_time
            y    += ydot * remaining_time + 0.5 * ay * remaining_time ** 2
            ydot += ay * remaining_time
            time_elapsed = 0.0   # reset for the next control segment

    return states


def integrate_control_2d_at_time(xi, con, t):
    """
    Integrate the control sequence up to a specific time t.

    Args:
        xi:  Initial state (x, y, xdot, ydot).
        con: Control sequence [[accel_vec, duration], ...].
        t:   Target time.

    Returns:
        State (x, y, xdot, ydot) at time t.
    """
    x, y, xdot, ydot = xi
    elapsed_time = 0.0

    for c in con:
        ax, ay = c[0]
        duration = c[1]

        if elapsed_time + duration >= t:
            remaining_time = t - elapsed_time
            x    += xdot * remaining_time + 0.5 * ax * remaining_time ** 2
            xdot += ax * remaining_time
            y    += ydot * remaining_time + 0.5 * ay * remaining_time ** 2
            ydot += ay * remaining_time
            return (x, y, xdot, ydot)

        x    += xdot * duration + 0.5 * ax * duration ** 2
        xdot += ax * duration
        y    += ydot * duration + 0.5 * ay * duration ** 2
        ydot += ay * duration
        elapsed_time += duration

    return (x, y, xdot, ydot)


def integrate_control_nd(xinit, con):
    """Integrate an n-dimensional control sequence to its terminal state."""
    n  = int(0.5 * len(xinit))
    xn = list(xinit)
    for c in con:
        for i in range(n):
            xn[i]     += xn[n + i] * c[1] + 0.5 * c[0][i] * c[1] ** 2
            xn[n + i] += c[0][i] * c[1]
    return xn


# ══════════════════════════════════════════════════════════════════════════════
#  MERGE
# ══════════════════════════════════════════════════════════════════════════════
def merge_vector_scalar_controls(c1, c2):
    """
    Merge a vector control (c1) with a scalar control (c2) that share the same
    total duration. Splits segments at every boundary of either sequence.
    """
    tt = control_time(c1)
    if tt == 0.0:
        return []

    # Terminal times for each segment in both sequences
    ts1, t = [], 0.0
    for cs in c1:
        t += cs[1]
        ts1.append(t)

    ts2, t = [], 0.0
    for cs in c2:
        t += cs[1]
        ts2.append(t)

    c   = []
    i   = j = 0
    t   = 0.0

    while i < len(ts1) and j < len(ts2):
        if abs(ts1[i] - ts2[j]) <= time_epsilon:
            dt = ts1[i] - t
            c.append([c1[i][0] + [c2[j][0]], dt])
            t = ts1[i] # Snap t to the exact boundary
            i += 1
            j += 1
        elif ts1[i] < ts2[j]:
            dt = ts1[i] - t
            c.append([c1[i][0] + [c2[j][0]], dt])
            t = ts1[i]
            i += 1
        else:
            dt = ts2[j] - t
            c.append([c1[i][0] + [c2[j][0]], dt])
            t = ts2[j]
            j += 1

    if abs(tt - control_time(c)) > 0.00001:
        print("control merge error.  Times don't match")

    return c

def merge_scalar_controls(cvec):
    """
    Merge a list of scalar control sequences into a single vector control.
    Each scalar sequence must span the same total duration.
    """
    cv = []
    for c in cvec[0]:
        cv.append([[c[0]], c[1]])
    cm = merge_vector_scalar_controls(cv, cvec[1])
    for i in range(2, len(cvec)):
        cm = merge_vector_scalar_controls(cm, cvec[i])
    return cm


# ══════════════════════════════════════════════════════════════════════════════
#  BANG-BANG PRIMITIVES  (internal helpers)
# ══════════════════════════════════════════════════════════════════════════════

def bang_bang_optimal(ix, iv, gx, gv, umin=-1.0, umax=1.0):
    """
    Time-optimal 1-D bang-bang control (NO velocity limit).
    Used internally by trapezoidal_optimal to find the unconstrained solution.
    Output: [[u1, t1], [u2, t2]]  (zero-duration segments omitted).
    """
    if ix == gx and iv == gv:
        return []

    invmin = 1 / umin
    invmax = 1 / umax
    c1 = ix - gx - 0.5 * (invmin * iv * iv - invmax * gv * gv)
    a1 = 0.5 * (invmin - invmax)
    s1 = -4.0 * a1 * c1

    c2 = ix - gx - 0.5 * (invmax * iv * iv - invmin * gv * gv)
    s2 =  4.0 * a1 * c2

    t1 = t1b = 1.0e20
    t2 = t2b = 1.0e20
    u1 = u1b = umin
    u2 = u2b = umax

    if s1 >= 0:
        xdot = sqrt(s1) / (2.0 * a1)
        t1 = invmin * (xdot - iv)
        t2 = invmax * (gv - xdot)
        u1, u2 = umin, umax

    if s2 >= 0:
        xdot = -sqrt(s2) / (2.0 * a1)
        t1b = invmax * (xdot - iv)
        t2b = invmin * (gv - xdot)
        u1b, u2b = umax, umin

    # Numerical cleanup
    for val in [t1, t2, t1b, t2b]:
        pass   # individual fix-ups below
    if fabs(t1)  < time_epsilon: t1  = 0.0
    if fabs(t2)  < time_epsilon: t2  = 0.0
    if fabs(t1b) < time_epsilon: t1b = 0.0
    if fabs(t2b) < time_epsilon: t2b = 0.0

    if (t1b + t2b < t1 + t2 and t1b >= 0.0 and t2b >= 0.0) or t1 < 0.0 or t2 < 0.0:
        t1, t2, u1, u2 = t1b, t2b, u1b, u2b

    if t1 == 0.0:
        return [[u2, t2]]
    if t2 == 0.0:
        return [[u1, t1]]
    return [[u1, t1], [u2, t2]]


def bang_bang_scaled(ix, iv, gx, gv, tf, umin=-1.0, umax=1.0):
    """
    Stretch a bang-bang control to a fixed duration tf.
    Returns [] on failure (tf shorter than optimal, or no feasible solution).
    """
    co  = bang_bang_optimal(ix, iv, gx, gv, umin, umax)
    tfo = control_time(co)

    if tfo > tf:
        print("bang_bang_scaled: requested final time is less than time optimal")
        return []

    u1  = umax
    num = gx - ix - (iv + gv) * tf * 0.5
    den = (iv - gv) * 0.5 + tf * u1 * 0.5

    if fabs(den) < float_epsilon:
        return []
    t1 = num / den

    if t1 < 0:
        u1  = umin
        den = (iv - gv) * 0.5 + tf * u1 * 0.5
        if fabs(den) < float_epsilon:
            return []
        t1 = num / den

    tden = tf - t1
    if fabs(tden) < float_epsilon:
        return []
    u2 = ((gv - iv) - u1 * t1) / tden

    if u2 > umax + time_epsilon or u2 < umin - time_epsilon:
        return []

    return [[u1, t1], [u2, tf - t1]]


def bang_bang_hard_stop(ix, iv, gx, gv, umin=-1.0, umax=1.0):
    """Brake to zero, then bang-bang to goal (no velocity limit)."""
    if iv > 0:
        ux = umin
    else:
        ux = umax
    s  = -iv / ux
    sx = ix + iv * s + 0.5 * ux * s * s
    d  = bang_bang_optimal(sx, 0.0, gx, gv, umin, umax)
    c  = []
    if s > 0.0:
        c.append([ux, s])
    c += d
    return c


def bang_bang_hard_stop_wait(ix, iv, gx, gv, tf, umin=-1.0, umax=1.0):
    """Brake to zero, wait if needed, then bang-bang to goal (no velocity limit)."""
    c  = bang_bang_hard_stop(ix, iv, gx, gv, umin, umax)
    tt = control_time(c)

    if tt > tf:
        return []

    if tf > tt:
        if iv == 0.0:
            c.insert(0, [0.0, tf - tt])
        else:
            c.insert(1, [0.0, tf - tt])

    return c


# ══════════════════════════════════════════════════════════════════════════════
# TEST
# ══════════════════════════════════════════════════════════════════════════════

def test_trapezoidal_steer():
    """
    Verify that time_optimal_steer_2d reaches the goal and respects vel limits.
    Runs a small random battery and prints any failures.
    """
    acc  = 2.0
    vmax = 1.5
    umin_2d = (-acc, -acc)
    umax_2d = ( acc,  acc)
    vmin_2d = (-vmax, -vmax)
    vmax_2d = ( vmax,  vmax)

    failures = 0
    for _ in range(500):
        xinit = (
            random.uniform(-5, 5), random.uniform(-5, 5),
            random.uniform(-vmax, vmax), random.uniform(-vmax, vmax),
        )
        xgoal = (
            random.uniform(-5, 5), random.uniform(-5, 5),
            0.0, 0.0,
        )

        con = time_optimal_steer_2d(xinit, xgoal, umin_2d, umax_2d, vmin_2d, vmax_2d)
        xf  = integrate_control_2d(xinit, con)

        pos_err = fabs(xf[0] - xgoal[0]) + fabs(xf[1] - xgoal[1])
        vel_err = fabs(xf[2] - xgoal[2]) + fabs(xf[3] - xgoal[3])

        # Check velocity limit along the trajectory
        states = integrate_control_2d_timestep(xinit, con, timestep=0.01)
        viol = any(
            fabs(s[2]) > vmax + 0.01 or fabs(s[3]) > vmax + 0.01
            for s in states
        )

        if pos_err > 0.01 or vel_err > 0.01 or viol:
            failures += 1
            print(f"FAIL  xinit={xinit}  xgoal={xgoal}  "
                  f"pos_err={pos_err:.4f}  vel_err={vel_err:.4f}  viol={viol}")

    print(f"test_trapezoidal_steer: {500 - failures}/500 passed")



if __name__ == "__main__":
    test_trapezoidal_steer()

    import matplotlib.pyplot as plt

    def plot_trajectory_and_profile(xinit, xgoal, con, timestep=0.01):
        """
        Simulates the control sequence and plots the 2D path, position over time, 
        and velocity over time.
        """
        # 1. Simulate the trajectory using your existing integrator
        states = integrate_control_2d_timestep(xinit, con, timestep)
    
        # 2. Unpack the state data
        times = [i * timestep for i in range(len(states))]
        x_vals = [s[0] for s in states]
        y_vals = [s[1] for s in states]
        xdot_vals = [s[2] for s in states]
        ydot_vals = [s[3] for s in states]
    
        # 3. Set up the plotting canvas
        fig, axs = plt.subplots(1, 3, figsize=(16, 5))
    
        # --- Plot A: 2D Spatial Trajectory ---
        axs[0].plot(x_vals, y_vals, label='Robot Path', color='purple', linewidth=2)
        axs[0].scatter([xinit[0]], [xinit[1]], color='green', s=100, label='Start', zorder=5)
        axs[0].scatter([xgoal[0]], [xgoal[1]], color='red', marker='X', s=100, label='Goal', zorder=5)
        axs[0].set_title('2D Field Trajectory')
        axs[0].set_xlabel('X Position (m)')
        axs[0].set_ylabel('Y Position (m)')
        axs[0].axis('equal')  # Forces 1:1 aspect ratio so curves look realistic
        axs[0].grid(True, linestyle='--', alpha=0.7)
        axs[0].legend()

        # --- Plot B: Position vs. Time ---
        axs[1].plot(times, x_vals, label='X Position', color='blue', linewidth=2)
        axs[1].plot(times, y_vals, label='Y Position', color='orange', linewidth=2)
        axs[1].axhline(xgoal[0], color='blue', linestyle=':', alpha=0.5, label='X Goal')
        axs[1].axhline(xgoal[1], color='orange', linestyle=':', alpha=0.5, label='Y Goal')
        axs[1].set_title('Position Profile')
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('Position')
        axs[1].grid(True, linestyle='--', alpha=0.7)
        axs[1].legend()

        # --- Plot C: Velocity vs. Time (Trapezoids!) ---
        axs[2].plot(times, xdot_vals, label='X Velocity', color='blue', linewidth=2)
        axs[2].plot(times, ydot_vals, label='Y Velocity', color='orange', linewidth=2)
        axs[2].set_title('Velocity Profile')
        axs[2].set_xlabel('Time (s)')
        axs[2].set_ylabel('Velocity')
        axs[2].grid(True, linestyle='--', alpha=0.7)
        axs[2].legend()

        plt.tight_layout()
        plt.show()

    # Define constraints
    vmax = 2.0
    acc = 1.5
    umin_2d = (-acc, -acc)
    umax_2d = (acc, acc)
    vmin_2d = (-vmax, -vmax)
    vmax_2d = (vmax, vmax)

    xinit = (-4.0, 2.0, 0.5, -1.0)
    xgoal = (3.0, -3.0, -1.0, 0.5)

    con = time_optimal_steer_2d(xinit, xgoal, umin_2d, umax_2d, vmin_2d, vmax_2d)

    plot_trajectory_and_profile(xinit, xgoal, con)
