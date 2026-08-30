import config
import numpy as np
from robot_math.pose import Pose

from dataclasses import dataclass


@dataclass
class LinearAxisStateObject:

    # TODO: Define axis in parameter?
    def __init__(self, kinematics):

        # World origin to robot base at zero joints transform
        self._kinematics = kinematics

        self._joint_state = np.zeros(config.N_LIN_JNT, dtype=np.float64)        # Joint vector (=motor vector).
        self._jacobian = np.array([[1., 0., 0.]], dtype=np.float64).T     # Jacobian (constant). Column vector.
        # self._position = np.zeros(3, dtype=np.float64)
        # TODO: Pose doesn't support arbitrary orientation.
        self._pose = Pose.identity()  # Robot base in world frame

        # Initialize state
        self.joint_state = self._joint_state

    @property
    def joint_state(self) -> np.ndarray:
        return self._joint_state.copy()

    @property
    def jacobian(self) -> np.ndarray:
        return self._jacobian.copy()

    # @property
    # def position(self) -> np.ndarray:
    #     return self._position.copy()
    @property
    def pose(self) -> Pose:
        return self._pose

    @joint_state.setter
    def joint_state(self, jnt_vec: np.ndarray):
        self._joint_state = jnt_vec.copy()
        # self._position = self._kinematics.forward(jnt_vec)
        self._pose.position = self._kinematics.forward(jnt_vec)  # Update operational space coordinates.


class LinearAxisState:

    def __init__(self, kinematics):

        self.mcu = LinearAxisStateObject(kinematics=kinematics)
        self.queued = LinearAxisStateObject(kinematics=kinematics)
        self.planned = LinearAxisStateObject(kinematics=kinematics)
