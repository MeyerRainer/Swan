import config
from backend.manipulator_state import ManipulatorState
from backend.linear_axis_state import LinearAxisState
from robot_math.pose import Pose

import numpy as np
from enum import Enum, auto

# GRBL states
class ControllerState(Enum):
    IDLE = auto()
    ALARM = auto()
    CHECK = auto()
    HOMING = auto()
    CYCLE = auto()
    HOLD = auto()
    SAFETY_DOOR = auto()
    MOTION_CANCEL = auto()


class SystemState:

    def __init__(self, manipulator_state: ManipulatorState, linear_axis_state: LinearAxisState):

        self._manipulator: ManipulatorState = manipulator_state  # 3 states
        self._linear_axis: LinearAxisState = linear_axis_state  # 3 states
        self.grbl = ControllerState

    @property
    def mcu_motors(self):
        mcu_mot_vec: np.ndarray = np.zeros(8)
        mcu_mot_vec[:6] = self._manipulator.mcu.motor_state
        mcu_mot_vec[6:7] = self._linear_axis.mcu.joint_state  # =motor_state
        return mcu_mot_vec

    @property
    def queue_motors(self):
        queued_mot_vec: np.ndarray = np.zeros(8)
        queued_mot_vec[:6] = self._manipulator.queued.motor_state
        queued_mot_vec[6:7] = self._linear_axis.queued.joint_state  # =motor_state
        return queued_mot_vec

    @property
    def mcu_joints(self):
        mcu_jnt_vec: np.ndarray = np.zeros(8, dtype=np.float64)
        mcu_jnt_vec[:6] = self._manipulator.mcu.joint_state
        mcu_jnt_vec[6:7] = self._linear_axis.mcu.joint_state
        return mcu_jnt_vec

    @property
    def mcu_pose(self):
        return self._manipulator.mcu.pose

    @property
    def queued_pose(self):
        return self._manipulator.queued.pose

    @property
    def mcu_jacobian(self, scale_rotation: bool = False) -> np.ndarray:
        J = np.zeros((6, 7), dtype=np.float64)
        J[:6, :6] = self._manipulator.mcu.jacobian
        J[:3, 6] = self._linear_axis.mcu.jacobian
        if scale_rotation:
            J[3:, :] *= config.CHAR_LEN
        return J

    @property
    def queued_jacobian(self, scale_rotation: bool = False) -> np.ndarray:
        J = np.zeros((6, 7), dtype=np.float64)
        J[:6, :6] = self._manipulator.queued.jacobian
        J[:3, 6] = self._linear_axis.queued.jacobian
        if scale_rotation:
            J[3:, :] *= config.CHAR_LEN
        return J

    # @property
    # def world_pose(self):
        # world_pose: Pose = self._manipulator.mcu.ops_state.relative_to(self._linear_axis.mcu.pose)

# class State:
#
#     def __init__(self):
#
#         self._mcu = SystemState()
#         self._queued = SystemState()
#         self._planned = SystemState()
#
#     @property
#     def mcu(self):
#         return self._mcu
#
#     @property
#     def queued(self):
#         return self._queued
#
#     @property
#     def planned(self):
#         return self._planned