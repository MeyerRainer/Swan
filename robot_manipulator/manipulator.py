""" Class for 6 axis manipulator.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from robot_math.quaternion import Quaternion
from robot_manipulator.manipulator_state import ManipulatorState
from robot_manipulator.kinematics.anthropomorphic_spherical_wrist import *
from typing import Tuple, Optional
import numpy as np


class JointSolution(Enum):
    OK = auto()
    J1_NEGATIVE_LIMIT = auto()
    J1_POSITIVE_LIMIT = auto()
    J2_NEGATIVE_LIMIT = auto()
    J2_POSITIVE_LIMIT = auto()
    J3_NEGATIVE_LIMIT = auto()
    J3_POSITIVE_LIMIT = auto()
    J4_NEGATIVE_LIMIT = auto()
    J4_POSITIVE_LIMIT = auto()
    J5_NEGATIVE_LIMIT = auto()
    J5_POSITIVE_LIMIT = auto()
    J6_NEGATIVE_LIMIT = auto()
    J6_POSITIVE_LIMIT = auto()


class MotorSolution(Enum):
    OK = auto()
    M1_NEGATIVE_LIMIT = auto()
    M1_POSITIVE_LIMIT = auto()
    M2_NEGATIVE_LIMIT = auto()
    M2_POSITIVE_LIMIT = auto()
    M3_NEGATIVE_LIMIT = auto()
    M3_POSITIVE_LIMIT = auto()
    M4_NEGATIVE_LIMIT = auto()
    M4_POSITIVE_LIMIT = auto()
    M5_NEGATIVE_LIMIT = auto()
    M5_POSITIVE_LIMIT = auto()
    M6_NEGATIVE_LIMIT = auto()
    M6_POSITIVE_LIMIT = auto()


# class IkSolution(Enum):
#     OK = auto()


@dataclass
class MotionQueryResponse:
    """ Manipulators response to a motion query. Fail if::
    -Inverse kinematics fail (point out of reach)
    -Joint value out of range
    -Motor value out of range
    On success:
    -

    """
    motion_accepted: bool
    ik_solution: IkSolution         #
    joint_solution: JointSolution   # Which joint out of range
    motor_solution: MotorSolution   # Which motor out of range
    motor_vector: np.ndarray        # New values for motors
    # joint_vector: np.ndarray
    # tool_pose: Pose


class Manipulator:

    def __init__(self):

        self.kinematics = ASWKinematics(DH=config.DH_TABLE)

        self.state = ManipulatorState(self.kinematics)

        # TODO: Collision model.

    @staticmethod
    def _ensure_motor_move(mot_vec: np.ndarray) -> Optional[np.ndarray]:
        """ Execute movement in motor space
        @param mot_vec: 6-vector of absolute motor coordinates in radians.
        @return: Same vector back if accepted, None otherwise
        """

        # Check limits
        mot_vec_deg = np.rad2deg(mot_vec)
        for idx in range(N_REV_JNT):
            if not MOTOR_LIMITS[f"M{idx+1}_MIN"] <= mot_vec_deg[idx] <= MOTOR_LIMITS[f"M{idx+1}_MAX"]:
                print(f"Revolute motor {idx+1} out of range with value of {mot_vec_deg[idx]} degrees.")
                return None

        return mot_vec.copy()

    def _ensure_joint_move(self, jnt_vec: np.ndarray) -> np.ndarray | None:
        """ Execute movement in joint space.
        @param jnt_vec: 6-vector of absolute joint coordinates in radians.
        @return: 6-vector of absolute motor coordinates if motion accepted, None otherwise
        """
        jnt_vec_abs = jnt_vec.copy()

        # Check limits
        angle_deg = np.rad2deg(jnt_vec_abs)  # deg
        for idx in range(N_REV_JNT):
            if not JOINT_LIMITS[f"J{idx+1}_MIN"] <= angle_deg[idx] <= JOINT_LIMITS[f"J{idx+1}_MAX"]:
                print(f"Revolute joint out of range")
                return None

        # Compute motor coordinates and propagate motion command downwards
        return self._ensure_motor_move(self.kinematics.jnt2mot(jnt_vec_abs))

    # ==================================== Public interface =======================================

    def reset(self):
        """ Reset internal state """
        self.state.mcu.motor_state = np.zeros(N_REV_JNT, dtype=np.float64)
        self.state.queued.motor_state = np.zeros(N_REV_JNT, dtype=np.float64)
        self.state.planner.motor_state = np.zeros(N_REV_JNT, dtype=np.float64)

    def move_motors(self, mcu: Optional[np.ndarray] = None, queued: Optional[np.ndarray] = None, planned: Optional[np.ndarray] = None) -> None:
        """ Update internal manipulator state with a pre-verified motor vector.
        :param mcu: Motor vector for mcu state, absolute radians.
        :param queued: Motor vector for queued state, absolute radians.
        :param planned: Motor vector for planned state, absolute radians.
        """
        if mcu is not None:
            self.state.mcu.motor_state = mcu

        if queued is not None:
            self.state.mcu.motor_state = queued

        if planned is not None:
            self.state.mcu.motor_state = planned

    def request_motor_move(self, mot_vec: np.ndarray, degrees: bool = False) -> Optional[np.ndarray]:
        """ Request move in motor space. Returns motor vector if move can be executed. """
        mot_pos_absolute = mot_vec.copy()
        if degrees:
            mot_pos_absolute = np.deg2rad(mot_pos_absolute)
        return self._ensure_motor_move(mot_pos_absolute)

    def request_joint_move(self, jnt_vec: np.ndarray, degrees: bool = False) -> Optional[np.ndarray]:
        """ Request move in joint space. Returns motor vector if move can be executed. """
        jnt_pos_absolute = jnt_vec.copy()
        if degrees:
            jnt_pos_absolute = np.deg2rad(jnt_pos_absolute)
        return self._ensure_joint_move(jnt_pos_absolute)

    def request_cartesian_move(self, target_pose: Pose) -> Optional[np.ndarray]:
        """ Requests motor space interpolated move to given posture.
        :param target_pose: Target pose.
        :return: Motor vector if move can be executed.
        """
        # Move too short
        if self.state.queued.ops_state.is_close(target_pose):
            return None

        current_jnt_vec: np.ndarray = self.state.queued.joint_state
        ik_sol: IkSolution = self.kinematics.inverse(target_pose, prev_jnt_vec=current_jnt_vec)
        if not ik_sol.success:
            print(f"move_ops_lin: IK fail. Joints={ik_sol.joint_limits}, Singularity={ik_sol.singularity}")
            return None
        jnt_vec: np.ndarray = ik_sol.joint_solution

        # Propagate motion command forwards
            # Propagate motion command forwards
        mot_vec = self._ensure_joint_move(jnt_vec)
        if mot_vec is None:
            return None

        return mot_vec

    def request_cartesian_linear_move(self, target_pose: Pose, speed_linear: float = None, speed_angular: float = None,
                                      segment_length_m: float = 0.001, segment_size_rad: float = 0.0035) -> Optional[Tuple[np.ndarray, float]]:
        """ Linear move in operational space  coordinates.
        :param target_pose: Target 6D-Pose in manipulator frame
        :param speed_linear: m/s, linear speed. By default, this is used.
        :param speed_angular: rad/s, rotational speed. Used if no linear speed is given.
        :param segment_length_m: m, Length of translational segment.
        :param segment_size_rad: rad, Size of rotational segment.
        :return: True if move was executed
        """
        current_pose: Pose = self.state.queued.ops_state

        if current_pose.is_close(target_pose):
            return None

        # Translational error, meters
        tool_translation_dist = current_pose.distance(target_pose)
        n_segments_lin = int(np.ceil(tool_translation_dist / segment_length_m))

        # Rotational error, radians
        tool_rotation_dist = current_pose.angle(target_pose)
        n_segments_ang = int(np.ceil(tool_rotation_dist / segment_size_rad))

        # Choose whether rotation or translation determines segment count
        n_segments = max(n_segments_lin, n_segments_ang)

        # Compute movement time (seconds)
        move_time = 0.
        if speed_linear is not None:
            move_time = max(tool_translation_dist / speed_linear, move_time)
        if speed_angular is not None:
            move_time = max(tool_rotation_dist / speed_angular, move_time)
        if move_time == 0:
            raise ValueError("Linear movement needs speed specified.")

        segment_time = move_time / n_segments  # Seconds

        current_jnt_vec = self.state.queued.joint_state
        mot_vecs = np.zeros((n_segments, N_REV_JNT))
        for idx in range(n_segments):
            t = (1+idx) / n_segments  # Interpolation parameter in range [0, 1]
            interp_pose = current_pose.interpolate(target_pose, t)
            ik_sol: IkSolution = self.kinematics.inverse(interp_pose, prev_jnt_vec=current_jnt_vec)
            if not ik_sol.success:
                print(f"move_ops_lin: IK fail.")
                return None
            interp_jnt_vec = ik_sol.joint_solution
            current_jnt_vec = interp_jnt_vec.copy()

            # Propagate motion command forwards
            mot_vec = self._ensure_joint_move(interp_jnt_vec)
            if mot_vec is None:
                return None

            mot_vecs[idx] = mot_vec

        # All interpolation points computed successfully
        return mot_vecs, segment_time

    def translate_tool(self, direction_vec: tuple[int, int, int], distance: float, speed: float, frame: str) -> Tuple[np.ndarray, float] | None:
        """ Creates a pure translation along any axis in any frame.
        :param direction_vec: Translation axis, any length
        :param distance: Translation distance, meters
        :param speed: Translation speed, meters/second
        :param frame: Frame direction vector is described in. "World", "Base" or "Tool"
        """
        # Normalize direction vector
        direction_vec = np.array(direction_vec)
        unit_vec = direction_vec / np.linalg.norm(direction_vec)

        # Convert direction vector relative to World/Base frame. Compute end posture.
        end_pose: Pose = self.state.queued.ops_state
        if frame == "Base" or frame == "World":
            pass
        elif frame == "Tool":
            unit_vec = end_pose.rot_mat @ unit_vec
        else:
            raise ValueError("Unknown frame type")

        end_pose.position += distance * unit_vec

        # Compute motor vector list for end and intermediate postures
        return self.request_cartesian_linear_move(end_pose, speed_linear=speed)

    def rotate_tool(self, direction_vec: tuple[int, int, int], angle: float, speed: float, frame: str) -> np.ndarray | None:
        """ Creates a pure rotation around any axis in any frame.
        :param direction_vec: Rotation axis, any length
        :param angle: Rotation angle, radians
        :param speed: Rotation speed, radians/second
        :param frame: Frame respect to which direction vector is described. "World", "Base" or "Tool"
        """
        # Normalize direction vector
        direction_vec = np.array(direction_vec)
        unit_vec = direction_vec / np.linalg.norm(direction_vec)

        # Convert direction vector and angle to a quaternion
        quat_rot = Quaternion.from_axis_angle(tuple(unit_vec), angle)

        # End posture by rotating current posture
        end_pose: Pose = self.state.queued.ops_state
        if frame == "Base" or frame == "World":
            end_pose.quaternion = quat_rot * end_pose.quaternion
        elif frame == "Tool":
            end_pose.quaternion = end_pose.quaternion * quat_rot
        else:
            raise ValueError("Invalid frame")

        # Compute motor vectors for end and intermediate postures
        return self.request_cartesian_linear_move(end_pose, speed_angular=speed)
