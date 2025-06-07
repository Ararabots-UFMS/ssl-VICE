from dataclasses import dataclass
from typing import Optional


@dataclass
class Vector2D:
    x: float
    y: float

    def __neg__(self):
        return Vector2D(-self.x, -self.y)

    # Not returning the sum of vector mathematically, overloading the add operator to
    # format the vectors to be inputed in the BB steer.
    def __add__(self, other) -> list:
        return [self.x, self.y, other.x, other.y]

    def __getitem__(self, index):
        if index == 0:
            return self.x
        elif index == 1:
            return self.y
        else:
            raise IndexError("Index out of range. Use 0 for x, 1 for y.")


@dataclass
class State:
    position: Vector2D
    velocity: Vector2D
    acceleration: Optional[Vector2D] = None


@dataclass
class MoveConstraints:
    max_velocity: Vector2D
    max_acceleration: Vector2D
    min_velocity: Optional[Vector2D] = None
    min_acceleration: Optional[Vector2D] = None

    def __post_init__(self):
        if self.min_velocity is None:
            self.min_velocity = -self.max_velocity
        if self.min_acceleration is None:
            self.min_acceleration = -self.max_acceleration
