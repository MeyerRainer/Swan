""" State class for robot system composed of 6-DOF manipulator and 1-DOF linear axis.
State consists of three parts:
mcu: Physical state reported back by MCU.
queued: State based on motion commands waiting for MCU execution.
planned: State based on trajectory planners.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

from robot_system.manipulator_state import ManipulatorState, ManipulatorStateObject
from robot_system.linear_axis_state import LinearAxisState, LinearAxisStateObject
from config import N_REV_JNT, N_LIN_JNT, N_JNT

import numpy as np
from enum import Enum, auto


# GRBL state
class ControllerState(Enum):
    IDLE = auto()
    ALARM = auto()
    CHECK = auto()
    HOMING = auto()
    CYCLE = auto()
    HOLD = auto()
    SAFETY_DOOR = auto()
    MOTION_CANCEL = auto()


# Manipulator and linear axis states combined in system state.
class SystemStateObject:

    def __init__(self, manipulator_state_object: ManipulatorStateObject, linear_axis_state_object: LinearAxisStateObject):

        self.manipulator: ManipulatorStateObject = manipulator_state_object
        self.linear_axis: LinearAxisStateObject = linear_axis_state_object

    @property
    def motor_state_ctrl_units(self):
        mot_vec = np.zeros(8)
        mot_vec[:N_REV_JNT] = np.rad2deg(self.manipulator.motor_state)
        mot_vec[N_REV_JNT:N_JNT] = 1000 * self.linear_axis.joint_state  # =motor_state
        return mot_vec

    @motor_state_ctrl_units.setter
    def motor_state_ctrl_units(self, mot_vec_ctrl_units):
        """ Sets manipulator and linear axis state
        :param mot_vec_ctrl_units: 8-vector of motor coordinates in degrees and millimeters.
        :return: None
        """
        self.manipulator.motor_state = np.deg2rad(mot_vec_ctrl_units[:N_REV_JNT])
        self.linear_axis.joint_state = 0.001 * mot_vec_ctrl_units[N_REV_JNT:N_JNT]  # =motor state

    @property
    def base_jacobian(self) -> np.ndarray:
        J = np.zeros(6, N_JNT)
        J[:, :N_REV_JNT] = self.manipulator.jacobian
        J[:3, N_REV_JNT:N_JNT] = self.linear_axis.jacobian
        return J


class SystemState:

    def __init__(self, manipulator_state: ManipulatorState, linear_axis_state: LinearAxisState):

        self.controller = ControllerState

        self.mcu: SystemStateObject = SystemStateObject(manipulator_state.mcu, linear_axis_state.mcu)
        self.queued: SystemStateObject = SystemStateObject(manipulator_state.queued, linear_axis_state.queued)
        self.planned: SystemStateObject = SystemStateObject(manipulator_state.planner, linear_axis_state.planned)
