from dataclasses import dataclass
from typing import Optional
from movement_interfaces.msg import Vector2D as Vector2DMsg


@dataclass
class MovementCommand:
    robot_id: int
    target_pos: Optional[Vector2DMsg] = None

    @classmethod
    def from_msg(cls, msg):
        tp = getattr(msg, 'target_pos', None)
        if tp is None:
            tp = Vector2DMsg()
            tp.x = 0.0
            tp.y = 0.0
        return cls(robot_id=int(getattr(msg, 'robot_id', 0)), target_pos=tp)