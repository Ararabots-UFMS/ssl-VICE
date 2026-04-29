from dataclasses import dataclass, field

try:
    from system_interfaces.msg import VisionMessage as VisionMessageMsg, Robots
except ImportError:
    class VisionMessageMsg:
        def __init__(self):
            self.yellow_robots = []
            self.blue_robots = []

    class Robots:
        pass

@dataclass
class VisionMessage:
    yellow_robots: list = field(default_factory=list)
    blue_robots: list = field(default_factory=list)

    @classmethod
    def from_msg(cls, msg: VisionMessageMsg):
        return cls(
            yellow_robots=msg.yellow_robots if hasattr(msg, 'yellow_robots') else [],
            blue_robots=msg.blue_robots if hasattr(msg, 'blue_robots') else []
        )

    def to_msg(self) -> VisionMessageMsg:
        msg = VisionMessageMsg()
        msg.yellow_robots = self.yellow_robots
        msg.blue_robots = self.blue_robots
        return msg