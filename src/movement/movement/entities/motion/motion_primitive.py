from dataclasses import dataclass

from utils.math_util import Vector2D

from movement_interfaces.msg import (
    MotionPrimitive as MotionPrimitiveMsg
)


@dataclass
class MotionPrimitive:
    acceleration: Vector2D
    duration: float  # seconds

    def to_msg(self) -> MotionPrimitiveMsg:
        return MotionPrimitiveMsg(
            acceleration=self.acceleration.to_msg(), duration=float(self.duration)
        )

    @classmethod
    def from_msg(cls, msg: MotionPrimitiveMsg) -> "MotionPrimitive":
        return cls(
            acceleration=Vector2D.from_msg(msg.acceleration),
            duration=float(msg.duration),
        )

    def __getitem__(self, index):
        if index == 0:
            return self.acceleration[0], self.acceleration[1]
        elif index == 1:
            return self.duration
        else:
            raise IndexError("Index out of range. Use 0 for x, 1 for y.")
