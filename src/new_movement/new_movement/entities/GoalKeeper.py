from dataclasses import dataclass
from typing import Optional

@dataclass
class GoalKeeper:
    robot_id: Optional[int] = None

    @classmethod
    def from_service_respons(cls, resp):
        return cls(robot_id=resp.robot_id)

    def __bool__(self):
        return self.robot_id is not None