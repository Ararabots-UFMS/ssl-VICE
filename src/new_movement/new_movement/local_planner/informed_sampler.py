import numpy as np

from utils.math_util import Vector2D

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
