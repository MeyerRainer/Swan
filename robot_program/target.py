from robot_math.pose import Pose

from dataclasses import dataclass
import numpy as np


@dataclass
class Target:
    """ Robot target.

    Pose and joint values may both exist.
    Depending on the instruction only one may be used.
    """

    name: str = ""
    pose: Pose | None = None      # Operational space
    joints: np.ndarray | None = None  # Joint space

    tool: str = "tool0"
    frame: str = "world"
    speed: float = 100.0
    zone: float = 0.0
