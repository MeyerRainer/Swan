from backend.manipulator_state import ManipulatorState
from backend.linear_axis_state import LinearAxisState
import numpy as np
from robot_math.pose import Pose


class SystemState:

    def __init__(self, manipulator_state: ManipulatorState, linear_axis_state: LinearAxisState):

        self._manipulator: ManipulatorState = manipulator_state  # 3 states
        self._linear_axis: LinearAxisState = linear_axis_state  # 3 states

    @property
    def mcu_motors(self):
        mcu_mot_vec: np.ndarray = np.zeros(8)
        mcu_mot_vec[:6] = self._manipulator.mcu.motor_state
        mcu_mot_vec[6:7] = self._linear_axis.mcu.joint_state  # =motor_state
        return mcu_mot_vec

    @property
    def mcu_joints(self):
        mcu_jnt_vec: np.ndarray = np.zeros(8)
        mcu_jnt_vec[:6] = self._manipulator.mcu.joint_state
        mcu_jnt_vec[6:7] = self._linear_axis.mcu.joint_state
        return mcu_jnt_vec

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