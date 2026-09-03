import numpy as np


class TrajectorySampler:
    """
    Flattens a TrajectorySegment chain into arrays so a whole batch of times can be
    evaluated in one numpy pass.

    ``get_state`` walks the segment chain and re-sums every duration on every call, so
    sampling N instants costs O(N x primitives) Python calls. The collision checker
    samples the same trajectory tens of times for every candidate path it considers,
    which made that the hot loop of the planner. Flattening is O(primitives) once; each
    sample afterwards is an array index.

    The flattened form is a snapshot. Trajectories are mutated in place (``truncate``,
    ``connect``, ``relocate``), so build a sampler when you are about to sample and let
    it go, rather than caching one on the segment.
    """

    __slots__ = ("_t0", "_px", "_py", "_vx", "_vy", "_ax", "_ay", "duration")

    def __init__(self, segment) -> None:
        t0, px, py, vx, vy, ax, ay = [], [], [], [], [], [], []
        elapsed = 0.0
        x = y = dx = dy = 0.0

        node = segment
        first = True
        while node is not None:
            # Each segment restarts from its own declared initial state rather than the
            # running one: add_child only enforces continuity to 1e-3, and get_state
            # integrates from init_pos/init_vel, so this matches it exactly.
            x, y = float(node.init_pos.x), float(node.init_pos.y)
            dx, dy = float(node.init_vel.x), float(node.init_vel.y)
            if first:
                first = False

            for primitive in node.motion_path.motion_path:
                acceleration = primitive.acceleration
                duration = float(primitive.duration)
                t0.append(elapsed)
                px.append(x)
                py.append(y)
                vx.append(dx)
                vy.append(dy)
                ax.append(float(acceleration.x))
                ay.append(float(acceleration.y))

                x += dx * duration + 0.5 * float(acceleration.x) * duration ** 2
                y += dy * duration + 0.5 * float(acceleration.y) * duration ** 2
                dx += float(acceleration.x) * duration
                dy += float(acceleration.y) * duration
                elapsed += duration

            node = node.child

        if not t0:
            # A segment with no primitives (start == goal) still has to answer queries;
            # one zero-acceleration row holds the initial state for every t.
            if segment is not None:
                x, y = float(segment.init_pos.x), float(segment.init_pos.y)
                dx, dy = float(segment.init_vel.x), float(segment.init_vel.y)
            t0, px, py, vx, vy, ax, ay = [0.0], [x], [y], [dx], [dy], [0.0], [0.0]

        self._t0 = np.asarray(t0, dtype=float)
        self._px = np.asarray(px, dtype=float)
        self._py = np.asarray(py, dtype=float)
        self._vx = np.asarray(vx, dtype=float)
        self._vy = np.asarray(vy, dtype=float)
        self._ax = np.asarray(ax, dtype=float)
        self._ay = np.asarray(ay, dtype=float)
        self.duration = elapsed

    def _index(self, times: np.ndarray) -> tuple:
        clamped = np.clip(np.asarray(times, dtype=float), 0.0, self.duration)
        # side="right" - 1 puts an instant exactly on a boundary in the primitive that
        # starts there, with dt = 0. Continuity makes both answers identical.
        idx = np.searchsorted(self._t0, clamped, side="right") - 1
        np.clip(idx, 0, self._t0.size - 1, out=idx)
        return idx, clamped - self._t0[idx]

    def positions(self, times: np.ndarray) -> np.ndarray:
        """Positions at ``times`` as an (N, 2) array. Times outside are clamped."""
        idx, dt = self._index(times)
        return np.column_stack(
            (
                self._px[idx] + self._vx[idx] * dt + 0.5 * self._ax[idx] * dt ** 2,
                self._py[idx] + self._vy[idx] * dt + 0.5 * self._ay[idx] * dt ** 2,
            )
        )

    def velocities(self, times: np.ndarray) -> np.ndarray:
        """Velocities at ``times`` as an (N, 2) array."""
        idx, dt = self._index(times)
        return np.column_stack(
            (self._vx[idx] + self._ax[idx] * dt, self._vy[idx] + self._ay[idx] * dt)
        )

    def position_bounds(self) -> tuple:
        """
        Exact axis-aligned box the trajectory stays inside, as (min_x, min_y, max_x,
        max_y).

        Sampling to find the extent would only bound the samples; each primitive is a
        parabola, so its extremes on an axis are at the two endpoints or at the vertex
        where that axis' velocity crosses zero, and the vertex only counts when it
        falls inside the primitive. ``positions`` clamps out-of-range times onto the
        endpoints, so this bounds every query the sampler can answer.
        """
        starts = self._t0
        durations = np.empty_like(starts)
        durations[:-1] = np.diff(starts)
        durations[-1] = max(self.duration - starts[-1], 0.0)

        def axis_extent(p, v, a):
            end = p + v * durations + 0.5 * a * durations ** 2
            low = np.minimum(p, end)
            high = np.maximum(p, end)

            # Vertex of the parabola, kept only where it lies within the primitive.
            vertex_time = np.divide(-v, a, out=np.full_like(v, -1.0), where=a != 0.0)
            inside = (vertex_time > 0.0) & (vertex_time < durations)
            vertex = p + v * vertex_time + 0.5 * a * vertex_time ** 2
            low = np.where(inside, np.minimum(low, vertex), low)
            high = np.where(inside, np.maximum(high, vertex), high)
            return float(low.min()), float(high.max())

        min_x, max_x = axis_extent(self._px, self._vx, self._ax)
        min_y, max_y = axis_extent(self._py, self._vy, self._ay)
        return (min_x, min_y, max_x, max_y)
