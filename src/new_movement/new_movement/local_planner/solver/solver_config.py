from dataclasses import dataclass

from utils.math_util import Vector2D 


@dataclass
class SolverConfig:
    """Configuration for planning algorithms."""
    max_iterations: int = 20
    field_length: float = 12000.0
    field_width: float = 9000.0
    # TODO: mutable Vector2D instance as a dataclass field default. Works only because
    # this repo's ROS2-pinned Python (3.10) doesn't enforce the unhashable-default check;
    # raises ValueError at import time on Python >=3.11. Should use default_factory instead.
    max_velocity: Vector2D = Vector2D(2000.0, -2000.0)  # mm/s
    max_acceleration: Vector2D = Vector2D(1500.0, -1500.0) # mm/s
    continuity_threshold: float = 1e-3
    collision_time_step: float = 0.2