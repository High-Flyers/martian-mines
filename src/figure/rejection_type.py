from enum import Enum, auto


class RejectionType(Enum):
    NONE = 0
    COLOR = auto(),
    AREA = auto(),
    TYPE = auto()
