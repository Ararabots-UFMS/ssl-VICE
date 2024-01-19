from enum import Enum


class OpCodes(Enum):
    INVALID = -1
    STOP = 0
    SMOOTH = 2
    NORMAL = 4
    ORIENTATION_AVERAGE = 8
    SPIN_CW = 16
    SPIN_CCW = 32
    USE_FORWARD_HEAD = 64
    USE_BACKWARD_HEAD = 128

    def __eq__(self, other):
        return (other.value if isinstance(other, OpCodes) else other) == self.value

    def __or__(self, other):
        return (other.value if isinstance(other, OpCodes) else other) + self.value

    def __and__(self, other):
        return (other.value if isinstance(other, OpCodes) else other) & self.value

    def __rand__(self, other):
        return self.__and__(other)

    def __add__(self, other):
        return self.__or__(other)
    
    def __radd__(self, other):
        return self.__add__(other)
