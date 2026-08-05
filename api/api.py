from backend.robot_system import RobotSystem as Rs
import application_context
from robot_math.pose import Pose

from dataclasses import dataclass
from typing import List, Optional
import numpy as np

from robot_program_delete.condition import Condition

robot = application_context.robot_sys

@dataclass
class Target:

    """ Robot target.
    Pose and joint values may both exist.
    Depending on the instruction only one may be used.
    """

    name: str = ""
    pose: Optional[Pose] = None      # Operational space
    joints: Optional[np.ndarray] = None  # Joint space

    tool: str = "tool0"
    frame: str = "world"
    speed: float = 100.0
    zone: float = 0.0

target_1 = Target(name="Target1", joints=np.array([10., 0., 0., 0., 0., 0., 0., 0.]))

def move_joint(target: Target) -> None:
    Rs.move_jnt(jnt_vec=target.joints, speed=target.speed)

def move_pose_linear(target: Target) -> None:
    Rs.move_ops(ops_pose=target.pose, speed=target.speed)

def move_pose_joint(target: Target) -> None:
    ...

# =================================== Timing ==================================
def wait(condition: Condition) -> None:
    ...
