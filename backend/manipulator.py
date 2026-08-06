"""
Class for 6-8 axis manipulator with anthropomorphic arm and spherical wrist.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
import config
from config import *
from robot_math.pose import Pose
from robot_math.quaternion import Quaternion
from backend.manipulator_state import ManipulatorState
from backend.kinematics.anthropomorphic_spherical_wrist import ASWKinematics

from math import sqrt, fabs
from typing import Tuple
import numpy as np
from numpy import linalg as LA

np.set_printoptions(precision=3, suppress=True)

IK_SOLUTION = {
    "SUCCESS": 0,
    "J1_LIM_TRIG": 1,
    "J2_LIM_TRIG": 2,
    "J3_LIM_TRIG": 3,
    "J4_LIM_TRIG": 4,
    "J5_LIM_TRIG": 5,
    "J6_LIM_TRIG": 6,
    "J7_LIM_TRIG": 7,
    "SHOULDER_SINGULARITY": 10,
    "ELBOW_SINGULARITY": 11,
    "WRIST_SINGULARITY": 12,
    "WRIST_FLIPPED": 13,
}


class Manipulator:

    def __init__(self):

        # Default height: d1 + a2 + a3 = 0.325
        # Default length: a1 + d4 + d6 <=> 0.30+0.190+0.030=0.250

        self.kinematics = ASWKinematics(DH=config.DH_PARAMS)

        self.state = ManipulatorState(self.kinematics)

    # TODO: Move to state?
    def _compute_condition(self, frame: np.ndarray) -> np.ndarray:
        """
        @param frame:
        @return: 6-array of condition [x, y, z, rx, ry, rz]
        """
        jacobian_trans = self.state.mcu.jacobian[:3, :3]
        jacobian_rot = self.state.mcu.jacobian[3:, 3:6]
        JJT_trans_inv = LA.pinv(jacobian_trans @ jacobian_trans.T)
        JJT_rot_inv = LA.pinv(jacobian_rot @ jacobian_rot.T)
        condition_vec = np.zeros(6, dtype=np.float32)
        cond_x_denom = sqrt(fabs(frame[:, 0].T @ JJT_trans_inv @ frame[:, 0]))
        cond_y_denom = sqrt(fabs(frame[:, 1].T @ JJT_trans_inv @ frame[:, 1]))
        cond_z_denom = sqrt(fabs(frame[:, 2].T @ JJT_trans_inv @ frame[:, 2]))
        cond_rx_denom = sqrt(fabs(frame[:, 0].T @ JJT_rot_inv @ frame[:, 0]))
        cond_ry_denom = sqrt(fabs(frame[:, 1].T @ JJT_rot_inv @ frame[:, 1]))
        cond_rz_denom = sqrt(fabs(frame[:, 2].T @ JJT_rot_inv @ frame[:, 2]))
        eps = 1e-4
        condition_vec[0] = 1 / cond_x_denom if (cond_x_denom - eps) > 0 else 0
        condition_vec[1] = 1 / cond_y_denom if (cond_y_denom - eps) > 0 else 0
        condition_vec[2] = 1 / cond_z_denom if (cond_z_denom - eps) > 0 else 0
        condition_vec[3] = 1 / cond_rx_denom if (cond_rx_denom - eps) > 0 else 0
        condition_vec[4] = 1 / cond_ry_denom if (cond_ry_denom - eps) > 0 else 0
        condition_vec[5] = 1 / cond_rz_denom if (cond_rz_denom - eps) > 0 else 0
        return condition_vec

    @staticmethod
    def _move_mot(mot_vec: np.ndarray) -> np.ndarray | None:
        """ Execute movement in motor space
        @param mot_vec: 6-vector of absolute motor coordinates in radians.
        @return: Same vector back if accepted, None otherwise
        """

        # Check limits
        mot_vec_deg = np.rad2deg(mot_vec)
        for idx in range(N_REV_JNT):
            if not MOTOR_LIMITS[f"M{idx+1}_MIN"] <= mot_vec_deg[idx] <= MOTOR_LIMITS[f"M{idx+1}_MAX"]:
                print(f"Revolute motor value out of range")
                return None

        return mot_vec.copy()

    def _move_jnt(self, jnt_vec: np.ndarray) -> np.ndarray | None:
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
        return self._move_mot(self.kinematics.jnt2mot(jnt_vec_abs))

    # ---------------------------------------------------------------------------------------------
    # ------------------------------------ Public interface ---------------------------------------
    # ---------------------------------------------------------------------------------------------

    def update_state(self, mot_vec: np.ndarray):
        self.state.mcu.motor_state = mot_vec

    def reset(self):
        self.state.mcu.motor_state = np.zeros(N_REV_JNT)
        self.state.queued.motor_state = np.zeros(N_REV_JNT)
        self.state.planner.motor_state = np.zeros(N_REV_JNT)

    def move_mot(self, mot_vec: np.ndarray, degrees: bool = False) -> np.ndarray | None:
        mot_pos_absolute = mot_vec.copy()
        if degrees:
            mot_pos_absolute = np.deg2rad(mot_pos_absolute)
        return self._move_mot(mot_pos_absolute)

    def move_jnt(self, jnt_vec: np.ndarray, degrees: bool = False) -> np.ndarray | None:
        jnt_pos_absolute = jnt_vec.copy()
        if degrees:
            jnt_pos_absolute = np.deg2rad(jnt_pos_absolute)
        return self._move_jnt(jnt_pos_absolute)

    # TODO: Fix this
    # def move_ops(self, target: Pose, shoulder_flip: bool = False, elbow_down: bool = False, wrist_flip: bool = False) -> np.ndarray | None:
    #     """ Motor space interpolated motion to given posture.
    #     :param target:
    #     :param elbow_down: Bool, for i_kin to choose a specific solution
    #     :param shoulder_flip: Bool, for i_kin to choose a specific solution
    #     :param wrist_flip: Bool, for i_kin to choose a specific solution
    #     :return: True if motion was executed
    #     """
    #
    #     # Move too short
    #     # TODO: Necessary?
    #     if self.state.queued.ops_state.is_close(target):
    #         return None
    #
    #     target_jnt_vec, ik_sol = self.kinematics.inverse(target, prev_jnt_vec=self.state.queued.joint_state, shoulder_flip=shoulder_flip, elbow_down=elbow_down, wrist_flip=wrist_flip)
    #
    #     # TODO: Instead of returning None, return some datastructures that includes IK_SOLUTION
    #     if ik_sol != IK_SOLUTION["SUCCESS"]:
    #         print(f"move_ops: IK fail: {ik_sol}")
    #         return None
    #
    #     # Propagate motion request forwards
    #     return self._move_jnt(target_jnt_vec)

    def move_ops_lin(self, target_pose: Pose, speed_linear: float = None, speed_angular: float = None,
                     segment_length_m: float = 0.001, segment_size_rad: float = 0.0035) -> Tuple[np.ndarray, float] | None:
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
            print("No speed given")
            return None

        segment_time = move_time / n_segments  # Seconds

        current_jnt_vec = self.state.queued.joint_state
        mot_vecs = np.zeros((n_segments, N_REV_JNT))
        for idx in range(n_segments):
            t = (1+idx) / n_segments  # Interpolation parameter in range [0, 1]
            interp_pose = current_pose.interpolate(target_pose, t)
            interp_jnt_vec, ik_sol = self.kinematics.inverse(interp_pose, prev_jnt_vec=current_jnt_vec)
            if ik_sol != IK_SOLUTION['SUCCESS']:
                print(f"move_ops_lin: IK fail: {ik_sol}")
                return None

            current_jnt_vec = interp_jnt_vec.copy()

            # Propagate motion command forwards
            mot_vec = self._move_jnt(interp_jnt_vec)
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
        unit_vec = direction_vec / LA.norm(direction_vec)

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
        return self.move_ops_lin(end_pose, speed_linear=speed)

    def rotate_tool(self, direction_vec: tuple[int, int, int], angle: float, speed: float, frame: str) -> np.ndarray | None:
        """ Creates a pure rotation around any axis in any frame.
        :param direction_vec: Rotation axis, any length
        :param angle: Rotation angle, radians
        :param speed: Rotation speed, radians/second
        :param frame: Frame respect to which direction vector is described. "World", "Base" or "Tool"
        """
        # Normalize direction vector
        direction_vec = np.array(direction_vec)
        unit_vec = direction_vec / LA.norm(direction_vec)

        # Convert direction vector and angle to a quaternion
        # quat_rot = utils.dir_vec_angle2quat(unit_vec, angle)  # New orientation w.rot.t current orientation cur_Q_new
        quat_rot = Quaternion.from_axis_angle(tuple(unit_vec), angle)

        # End posture by rotating current posture
        end_pose: Pose = self.state.queued.ops_state
        if frame == "Base" or frame == "World":
            # new orientation w.r.t. world (base)
            end_pose.quaternion = quat_rot * end_pose.quaternion
        elif frame == "Tool":
            end_pose.quaternion = end_pose.quaternion * quat_rot
        else:
            raise ValueError("Invalid frame")

        # Compute G-code list for end and intermediate postures
        return self.move_ops_lin(end_pose, speed_angular=speed)
