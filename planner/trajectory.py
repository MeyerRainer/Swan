from dataclasses import dataclass
from typing import List, Optional
from enum import Enum, auto
from robot_math.pose import Pose


class SegmentType(Enum):

    LINE = auto()
    ARC = auto()


@dataclass
class Segment:

    type: SegmentType
    poses: List[Pose]
    speed: float


class Trajectory:

    def __init__(self, target_id: str):

        self.target_id: str = target_id
        self.segments: List[Segment] = []
        self.end_pose: Optional[Pose] = None
