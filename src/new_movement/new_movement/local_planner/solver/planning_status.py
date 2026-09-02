from enum import Enum, auto

class PlanningStatus(Enum):
    SUCCESS = auto()
    DIRECT_PATH = auto()
    BYPASS_FOUND = auto()
    FAILED = auto()
    RECOVERY = auto()