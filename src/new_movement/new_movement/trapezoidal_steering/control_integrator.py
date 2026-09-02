"""Double-integrator control integration."""


class ControlIntegrator:
    """Integrates control segments into position and velocity states."""

    def __init__(self):
        pass

    def two_dimensional(self, state, control):
        x, y, x_velocity, y_velocity = state
        for acceleration, duration in control:
            x += x_velocity * duration + 0.5 * acceleration[0] * duration ** 2
            x_velocity += acceleration[0] * duration
            y += y_velocity * duration + 0.5 * acceleration[1] * duration ** 2
            y_velocity += acceleration[1] * duration
        return x, y, x_velocity, y_velocity

    def two_dimensional_at_time(self, state, control, time):
        x, y, x_velocity, y_velocity = state
        elapsed = 0.0
        for acceleration, duration in control:
            step = min(duration, max(0.0, time - elapsed))
            x += x_velocity * step + 0.5 * acceleration[0] * step ** 2
            x_velocity += acceleration[0] * step
            y += y_velocity * step + 0.5 * acceleration[1] * step ** 2
            y_velocity += acceleration[1] * step
            elapsed += duration
            if elapsed >= time:
                return x, y, x_velocity, y_velocity
        return x, y, x_velocity, y_velocity

    def two_dimensional_timestep(self, state, control, timestep):
        states = [tuple(state)]
        for acceleration, duration in control:
            remaining = duration
            while remaining > 0:
                step = min(timestep, remaining)
                states.append(self.two_dimensional(states[-1], [(acceleration, step)]))
                remaining -= step
        return states

    def n_dimensional(self, state, control):
        dimension = len(state) // 2
        result = list(state)
        for acceleration, duration in control:
            for index in range(dimension):
                result[index] += result[dimension + index] * duration + 0.5 * acceleration[index] * duration ** 2
                result[dimension + index] += acceleration[index] * duration
        return result
