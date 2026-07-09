from dataclasses import dataclass
import numpy as np


@dataclass
class Target:
    """
    Robot target.

    Pose and joint values may both exist.
    Depending on the instruction only one may be used.
    """

    name: str = ""
    pose: np.ndarray | None = None      # [X Y Z W I J K]
    joints: np.ndarray | None = None    # [J1-J8]

    tool: str = "tool0"
    frame: str = "world"
    speed: float = 100.0
    zone: float = 0.0
