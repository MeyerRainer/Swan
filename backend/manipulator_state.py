""" State class for 6-DOF manipulator. State consists of three parts:
mcu: Physical state reported back by MCU.
queued: State based on motion commands waiting for MCU execution.
planned: State based on trajectory planners.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

from dataclasses import dataclass
from typing import List, Optional

from robot_math.pose import Pose
import numpy as np


@dataclass
class ManipulatorStateObject:

    def __init__(self, kinematics):

        self._kinematics = kinematics

        self._motor_state: np.ndarray = np.zeros(6, dtype=np.float64)    # Motor coordinates, radians
        self._joint_state: np.ndarray = np.zeros(6, dtype=np.float64)    # Joint coordinates, radians
        self._ops_state: Pose = Pose.identity()                                # 6D operational space posture
        self._jacobian: np.ndarray = np.zeros(6)
        self._link_poses: List[Pose] = []
        self._spring_poses: List[Pose] = []
        self._counter_weight_pose: Optional[Pose] = None
        self._sing_vals_translation: np.ndarray = np.zeros(3, dtype=np.float64)
        self._sing_vecs_translation: np.ndarray = np.zeros((3, 3), dtype=np.float64)
        self._sing_vals_rotation: np.ndarray = np.zeros(3, dtype=np.float64)
        self._sing_vecs_rotation: np.ndarray = np.zeros((3, 3), dtype=np.float64)

        # Initialize state
        self.motor_state = self._motor_state

    @property
    def motor_state(self) -> np.ndarray:
        return self._motor_state.copy()

    @property
    def joint_state(self) -> np.ndarray:
        return self._joint_state.copy()

    @property
    def ops_state(self) -> Pose:
        return self._ops_state.copy()

    @property
    def link_poses(self) -> List[Pose]:
        return self._link_poses.copy()

    @property
    def spring_poses(self) -> List[Pose]:
        return self._spring_poses.copy()

    @property
    def counter_weight_pose(self) -> Pose:
        return self._counter_weight_pose.copy()

    @property
    def jacobian(self) -> np.ndarray:
        return self._jacobian.copy()

    @property
    def singular_data_translation(self):
        return self._sing_vals_translation, self._sing_vecs_translation

    @property
    def singular_data_rotation(self):
        return self._sing_vals_rotation, self._sing_vecs_rotation

    @motor_state.setter
    def motor_state(self, mot_vec: np.ndarray):
        # Motor state is source of truth. Everything else derived from it.
        self._motor_state = mot_vec.copy()
        jnt_vec = self._kinematics.mot2jnt(mot_vec)
        self._joint_state = jnt_vec
        poses = self._kinematics.forward(jnt_vec)
        self._link_poses = poses["link_poses"]
        self._spring_poses = poses["spring_poses"]
        self._counter_weight_pose = poses["counter_weight_pose"]
        self._ops_state = self._link_poses[-1]
        self._jacobian = self._kinematics.jacobian(jnt_vec)

        # Singular values and vectors
        eig_vals_trans, self._sing_vecs_trans = np.linalg.eigh(self._jacobian[:3, :3] @ self._jacobian[:3, :3].T)
        eig_vals_rot, self._sing_vecs_rot = np.linalg.eigh(self._jacobian[3:, 3:6] @ self._jacobian[3:, 3:6].T)
        self._sing_vals_translation = np.sqrt(abs(eig_vals_trans))
        self._sing_vals_rotation = np.sqrt(abs(eig_vals_rot))


@dataclass
class ManipulatorState:

    def __init__(self, kinematics):

        self.mcu = ManipulatorStateObject(kinematics)      # Actual machine state based on MCU report
        self.queued = ManipulatorStateObject(kinematics)   # State based on motion commands queued one the MCU
        self.planner = ManipulatorStateObject(kinematics)  # State based on planner buffer
