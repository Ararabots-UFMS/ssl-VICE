from enum import Enum, auto

class PlanningStatus(Enum):
    SUCCESS = auto()
    DIRECT_PATH = auto()
    BYPASS_FOUND = auto()
    FAILED = auto()
    RECOVERY = auto()
    # Stops short of a goal another robot is standing on.
    PARTIAL = auto()