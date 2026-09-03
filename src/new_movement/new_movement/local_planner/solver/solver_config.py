from dataclasses import dataclass, field

from utils.math_util import Vector2D


@dataclass
class SolverConfig:
    """Configuration for planning algorithms."""
    max_iterations: int = 20
    field_length: float = 12000.0
    field_width: float = 9000.0
    # Bounds are magnitudes: MoveConstraints derives min = -max from them. Negative
    # values invert the bounds and break the trapezoidal steer (braking accelerates,
    # the velocity cap is never enforced), so __post_init__ rejects them.
    # default_factory also avoids sharing one mutable Vector2D across every instance,
    # which raises ValueError at import time on Python >= 3.11.
    max_velocity: Vector2D = field(default_factory=lambda: Vector2D(2000.0, 2000.0))  # mm/s
    max_acceleration: Vector2D = field(default_factory=lambda: Vector2D(1500.0, 1500.0)) # mm/s²
    continuity_threshold: float = 1e-3
    # How much faster a newly sampled bypass must be to replace the previous one.
    bypass_cost_margin: float = 0.15
    # How far past an obstacle's boundary to place an escape point. adaptDestination
    # returns the boundary itself, where isCollidingAt is still true.
    escape_margin: float = 20.0
    # Collision sampling step. At 2000 mm/s a 0.2 s step advances 400 mm between
    # samples — wider than an obstacle diameter, so the check can tunnel through it.
    # 0.04 s is ~80 mm of travel.
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
