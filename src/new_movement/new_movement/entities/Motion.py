from dataclasses import dataclass
from States import Vector2D


@dataclass
class MotionPrimitive:
    acceleration: Vector2D
    duration: float


@dataclass
class MotionPath:
    """Piecewise constant acceleration motion path"""

    motion_path: list[MotionPrimitive]

    def truncate(self, t: float) -> None:
        """Truncates the motion path by time"""
        elipsed_time = 0
        i = 0
        while self.motion_path[i].duration + elipsed_time < t:
            elipsed_time += self.motion_path[i].duration
            i += 1

        self.motion_path[i].duration = t - elipsed_time

        self.motion_path = self.motion_path[0 : i + 1]
