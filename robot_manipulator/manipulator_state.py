""" State class for 6-DOF manipulator. State consists of three parts:
mcu: Physical state reported back by MCU.
queued: State based on motion commands waiting for MCU execution.
planned: State based on trajectory planners.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from dataclasses import dataclass
from typing import List, Optional, Dict

import config
from qt_gui.viewport.scene.scene_node import SceneNode

from robot_math.pose import Pose
import numpy as np


class ManipulatorStateObject:

    def __init__(self, kinematics):

        self._kinematics = kinematics
        self._base_pose: Pose = Pose.from_position(config.BASE_OFFSET)  # Base w.r.t. world.
        self._motor_state: np.ndarray = np.zeros(6, dtype=np.float64)    # Motor coordinates, radians.
        self._joint_state: np.ndarray = np.zeros(6, dtype=np.float64)    # Joint coordinates, radians.
        self._ops_state: Pose = Pose.identity()                                # 6D operational space posture.
        self._jacobian: np.ndarray = np.zeros((6, 6), dtype=np.float64)
        self._jjt: np.ndarray = np.zeros((6, 6), dtype=np.float64)      # J*J^T Manipulability matrix.
        self._link_poses: List[Pose] = []
        self._spring_poses: List[Pose] = []
        self._counter_weight_pose: Optional[Pose] = None
        self._parallel_link_pose: Optional[Pose] = None
        self._sing_vals_translation: np.ndarray = np.zeros(3, dtype=np.float64)
        self._sing_vecs_translation: np.ndarray = np.zeros((3, 3), dtype=np.float64)
        self._sing_vals_rotation: np.ndarray = np.zeros(3, dtype=np.float64)
        self._sing_vecs_rotation: np.ndarray = np.zeros((3, 3), dtype=np.float64)

        self.link_nodes: Optional[Dict[str, SceneNode]] = {}  # SceneNodes for robot links

        # Initialize state
        self.motor_state = self._motor_state

    @property
    def base_pose(self) -> Pose:
        return self._base_pose.copy()

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
    def parallel_link_pose(self) -> Pose:
        return self._parallel_link_pose.copy()

    @property
    def jacobian(self) -> np.ndarray:
        return self._jacobian.copy()

    @property
    def singular_data_translation(self):
        return self._sing_vals_translation, self._sing_vecs_translation

    @property
    def singular_data_rotation(self):
        return self._sing_vals_rotation, self._sing_vecs_rotation

    @base_pose.setter
    def base_pose(self, pose: Pose) -> None:
        self._base_pose = pose.copy()

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
        self._parallel_link_pose = poses["parallel_link_pose"]
        self._ops_state = self._link_poses[-1]
        self._jacobian = self._kinematics.jacobian(jnt_vec)
        self._jjt_translation = self.jacobian[:3, :3] @ self.jacobian[:3, :3].T
        self._jjt_rotation = self.jacobian[3:, 3:] @ self.jacobian[3:, 3:].T

        self.update_link_visuals()

    # TODO: Consider optimizing implementation.
    def condition(self, frame: np.ndarray) -> np.ndarray:
        JJT_trans_inv = np.linalg.pinv(self._jjt_translation)
        JJT_rot_inv = np.linalg.pinv(self._jjt_rotation)
        condition_vec = np.zeros(6, dtype=np.float32)
        cond_x_denom = np.sqrt(np.fabs(frame[:, 0].T @ JJT_trans_inv @ frame[:, 0]))
        cond_y_denom = np.sqrt(np.fabs(frame[:, 1].T @ JJT_trans_inv @ frame[:, 1]))
        cond_z_denom = np.sqrt(np.fabs(frame[:, 2].T @ JJT_trans_inv @ frame[:, 2]))
        cond_rx_denom = np.sqrt(np.fabs(frame[:, 0].T @ JJT_rot_inv @ frame[:, 0]))
        cond_ry_denom = np.sqrt(np.fabs(frame[:, 1].T @ JJT_rot_inv @ frame[:, 1]))
        cond_rz_denom = np.sqrt(np.fabs(frame[:, 2].T @ JJT_rot_inv @ frame[:, 2]))
        eps = 1e-4
        condition_vec[0] = 1 / cond_x_denom if (cond_x_denom - eps) > 0 else 0
        condition_vec[1] = 1 / cond_y_denom if (cond_y_denom - eps) > 0 else 0
        condition_vec[2] = 1 / cond_z_denom if (cond_z_denom - eps) > 0 else 0
        condition_vec[3] = 1 / cond_rx_denom if (cond_rx_denom - eps) > 0 else 0
        condition_vec[4] = 1 / cond_ry_denom if (cond_ry_denom - eps) > 0 else 0
        condition_vec[5] = 1 / cond_rz_denom if (cond_rz_denom - eps) > 0 else 0
        return condition_vec

    def update_link_visuals(self):
        # base_pose = Pose.from_position(config.BASE_OFFSET)
        base_pose = self._base_pose
        if "L2" in self.link_nodes:
            self.link_nodes["L2"].pose = base_pose.compose(self.link_poses[0])
        if "L3" in self.link_nodes:
            self.link_nodes["L3"].pose = base_pose.compose(self.link_poses[1])
        if "L4" in self.link_nodes:
            self.link_nodes["L4"].pose = base_pose.compose(self.link_poses[2])
        if "L5" in self.link_nodes:
            self.link_nodes["L5"].pose = base_pose.compose(self.link_poses[3])
        if "L6" in self.link_nodes:
            self.link_nodes["L6"].pose = base_pose.compose(self.link_poses[4])
        # if "ToolFrame" in self.link_nodes:
        #     self.link_nodes["ToolFrame"].pose = base_pose.compose(self.link_poses[5].compose(Pose(config.TOOL_OFS)))
        if "PenHolder" in self.link_nodes:
            self.link_nodes["PenHolder"].pose = base_pose.compose(self.link_poses[5].compose(Pose(config.TOOL_OFS)))
        if "LCW" in self.link_nodes:
            self.link_nodes["LCW"].pose = base_pose.compose(self._counter_weight_pose)
        if "LPL" in self.link_nodes:
            self.link_nodes["LPL"].pose = base_pose.compose(self._parallel_link_pose)
        if "SpringDown" in self.link_nodes:
            self.link_nodes["SpringDown"].pose = base_pose.compose(self.spring_poses[0])

    def render_link_visuals(self, visible: bool):
        for link in self.link_nodes.items():
            link.visible = visible


@dataclass
class ManipulatorState:

    def __init__(self, kinematics):

        self.mcu = ManipulatorStateObject(kinematics)      # Actual machine state based on MCU report
        self.queued = ManipulatorStateObject(kinematics)   # State based on motion commands queued one the MCU
        self.planner = ManipulatorStateObject(kinematics)  # State based on planner buffer
