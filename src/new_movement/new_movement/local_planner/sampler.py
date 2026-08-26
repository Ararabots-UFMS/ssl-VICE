import numpy as np
from new_movement.entities.States import Vector2D

class InformedSampler:
    """Handles intelligent point sampling for bypass generation using NumPy."""
    
    def __init__(self, field_length: float, field_width: float, max_velocity: float):
        self.field_length = field_length
        self.field_width = field_width
        self.max_velocity = max_velocity

    def sample_near_axis(self, start: Vector2D, goal: Vector2D) -> Vector2D:
        """Samples a point with a bias towards the line connecting start and goal."""
        if np.random.random() > 0.3:
            midpoint = np.array([(start.x + goal.x) / 2, (start.y + goal.y) / 2])
            dist = start.distance(goal)
            sampled_point = np.random.normal(midpoint, dist / 4)
            return Vector2D(float(sampled_point[0]), float(sampled_point[1]))
        return self.sample_uniform()

    def sample_uniform(self) -> Vector2D:
        """Standard uniform field sampling."""
        x = np.random.uniform(-self.field_length / 2, self.field_length / 2)
        y = np.random.uniform(-self.field_width / 2, self.field_width / 2)
        return Vector2D(float(x), float(y))

    def sample_velocity(self) -> Vector2D:
        """Generates a random velocity within constraints."""
        v = self.max_velocity
        sampled_v = np.random.uniform(-v, v, size=2)
        return Vector2D(float(sampled_v[0]), float(sampled_v[1]))

    def sample_tangential_velocity(
        self, start: Vector2D, via: Vector2D, goal: Vector2D
    ) -> Vector2D:
        """
        Samples a speed along the tangent of a smooth path through the via point.

        Uniform sampling of the velocity square spends most samples on via velocities
        pointing away from the goal, which either fail or win on duration by accident and
        give the robot a different-looking path every cycle.
        """
        direction = None
        for leg in (via.subtract(start), goal.subtract(via)):
            if leg.size() > 1e-6:
                unit = leg.norm()
                direction = unit if direction is None else direction.add(unit)

        # Degenerate when the via lies beyond the goal on the same line: stop there.
        if direction is None or direction.size() < 1e-6:
            return Vector2D(0.0, 0.0)

        speed = float(np.random.uniform(0.0, self.max_velocity))
        return direction.norm().multiplyByScalar(speed)
