from dataclasses import dataclass, field

from utils.math_util import Vector2D


@dataclass
class SolverConfig:
    """Configuration for planning algorithms."""
    max_iterations: int = 20
    field_length: float = 12000.0
    field_width: float = 9000.0
    # Magnitudes: MoveConstraints derives min = -max from them, so a negative value
    # inverts the bounds and breaks the trapezoidal steer. Rejected in __post_init__.
    max_velocity: Vector2D = field(default_factory=lambda: Vector2D(2000.0, 2000.0))  # mm/s
    max_acceleration: Vector2D = field(default_factory=lambda: Vector2D(1500.0, 1500.0)) # mm/s²
    continuity_threshold: float = 1e-3
    # How much faster a newly sampled bypass must be to replace the previous one.
    bypass_cost_margin: float = 0.15
    # How far past an obstacle's boundary to place an escape point: adaptDestination
    # returns the boundary itself, where isCollidingAt is still true.
    escape_margin: float = 20.0
    # Collision sampling step, ~80 mm of travel at full speed.
    collision_time_step: float = 0.04

    def __post_init__(self):
        if self.max_velocity.x <= 0 or self.max_velocity.y <= 0:
            raise ValueError(
                f"max_velocity must be positive on both axes, got {self.max_velocity}"
            )
        if self.max_acceleration.x <= 0 or self.max_acceleration.y <= 0:
            raise ValueError(
                f"max_acceleration must be positive on both axes, got {self.max_acceleration}"
            )
