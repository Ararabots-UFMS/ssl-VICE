"""One-dimensional bang-bang control solver."""

from math import fabs, sqrt


class BangBangSolver:
    """Computes acceleration-limited one-dimensional control profiles."""

    def __init__(self, time_epsilon: float = 1.0e-7):
        self.time_epsilon = time_epsilon

    def duration(self, control):
        return sum(segment[1] for segment in control)

    def optimal(self, ix, iv, gx, gv, umin=-1.0, umax=1.0):
        if ix == gx and iv == gv:
            return []
        inv_min, inv_max = 1 / umin, 1 / umax
        coefficient = 0.5 * (inv_min - inv_max)
        cases = (
            (ix - gx - 0.5 * (inv_min * iv ** 2 - inv_max * gv ** 2), -1.0, 1.0, umin, umax),
            (ix - gx - 0.5 * (inv_max * iv ** 2 - inv_min * gv ** 2), 1.0, -1.0, umax, umin),
        )
        candidates = []
        for constant, discriminant_sign, velocity_sign, first_acceleration, second_acceleration in cases:
            discriminant = discriminant_sign * 4.0 * coefficient * constant
            if discriminant < 0:
                continue
            switching_velocity = velocity_sign * sqrt(discriminant) / (2.0 * coefficient)
            first_time = (switching_velocity - iv) / first_acceleration
            second_time = (gv - switching_velocity) / second_acceleration
            if first_time >= -self.time_epsilon and second_time >= -self.time_epsilon:
                candidates.append((max(0.0, first_time), max(0.0, second_time), first_acceleration, second_acceleration))
        if not candidates:
            return []
        first_time, second_time, first_acceleration, second_acceleration = min(candidates, key=lambda item: item[0] + item[1])
        result = []
        if first_time > self.time_epsilon:
            result.append([first_acceleration, first_time])
        if second_time > self.time_epsilon:
            result.append([second_acceleration, second_time])
        return result

    def scaled(self, ix, iv, gx, gv, final_time, umin=-1.0, umax=1.0):
        if self.duration(self.optimal(ix, iv, gx, gv, umin, umax)) > final_time:
            return []
        acceleration = umax
        numerator = gx - ix - (iv + gv) * final_time * 0.5
        denominator = (iv - gv) * 0.5 + final_time * acceleration * 0.5
        if fabs(denominator) < 1.0e-200:
            return []
        first_time = numerator / denominator
        if first_time < 0:
            acceleration = umin
            denominator = (iv - gv) * 0.5 + final_time * acceleration * 0.5
            if fabs(denominator) < 1.0e-200:
                return []
            first_time = numerator / denominator
        second_time = final_time - first_time
        if first_time < 0 or second_time < 0:
            return []
        second_acceleration = ((gv - iv) - acceleration * first_time) / second_time if second_time else 0
        if not umin <= second_acceleration <= umax:
            return []
        return [[acceleration, first_time], [second_acceleration, second_time]]
