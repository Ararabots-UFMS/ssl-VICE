from dataclasses import dataclass
from typing import Optional

from utils.math_util import Vector2D


@dataclass
class MotionState:
    position: Vector2D
    velocity: Vector2D
    acceleration: Optional[Vector2D] = None
