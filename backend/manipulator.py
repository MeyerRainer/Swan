"""
Class for 6-8 axis manipulator with anthropomorphic arm and spherical wrist.
Author: Rainer Meyer, rot.meyer494@gmail.com
"""
import config
from config import *
from backend import utils
from backend.pose import Pose

import math
from math import pi, sin, cos, tan, atan, sqrt, fabs
from typing import Tuple, Union, List, Literal
import numpy as np
from numpy import linalg as LA
from dataclasses import dataclass

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


# @dataclass
# class State:
#     status: str
#     mot_coords: np.ndarray[tuple[Literal[8]], np.dtype[np.float32]]
#     jnt_coords: np.ndarray[tuple[Literal[8]], np.dtype[np.float32]]
#     ops_coords: np.ndarray[tuple[Literal[7]], np.dtype[np.float32]]


class Manipulator:
    # Constructor
    def __init__(self):

        # self._state: State = State(
        #     "Uninitialized",
        #     np.zeros(8, dtype=np.float32),
        #     np.zeros(8, dtype=np.float32),
        #     np.zeros(7, dtype=np.float32),
        # )

        # Default height: d1 + a2 + a3 = 0.325
        # Default length: a1 + d4 + d6 <=> 0.30+0.190+0.030=0.250

        # Temporary coordinates. Needed if joint motions are generated before main state has been updated.
        self._temp_mot_coords = np.zeros(N_REV_JNT, dtype=np.float32)
        self._temp_jnt_coords = np.zeros(N_REV_JNT, dtype=np.float32)
        self._temp_ops_coords = Pose.identity()
        self._temp_base_jacobian: np.ndarray = np.zeros((6, N_REV_JNT), dtype=np.float32)  # Manipulator jacobian in world and base frame
        self._temp_tool_jacobian: np.ndarray = np.zeros((6, N_REV_JNT), dtype=np.float32)  # Manipulator jacobian in tool frame

        # Main manipulator state. Updated only through serial port status update from robot controller
        self._mot_coords = np.zeros(N_REV_JNT, dtype=np.float32)
        self._jnt_coords = np.zeros(N_REV_JNT, dtype=np.float32)
        self._ops_coords = Pose.identity()
        self._base_jacobian: np.ndarray = np.zeros((6, N_REV_JNT), dtype=np.float32)  # Manipulator jacobian in world and base frame
        self._tool_jacobian: np.ndarray = np.zeros((6, N_REV_JNT), dtype=np.float32)  # Manipulator jacobian in tool frame

        self._sing_vals_trans: np.ndarray = np.zeros(3, dtype=np.float32)
        self._sing_vecs_trans: np.ndarray = np.zeros((3, 3), dtype=np.float32)
        self._sing_vals_rot: np.ndarray = np.zeros(3, dtype=np.float32)
        self._sing_vecs_rot: np.ndarray = np.zeros((3, 3), dtype=np.float32)

        self.wrist_flip = False

        self._update_temp_state(self._temp_mot_coords)

    def print_state(self):
        """ Prints manipulator state """

        print(f"Motor coordinates:\n"
              f"M1: {self._mot_coords[0]:.3f}\n"
              f"M2: {self._mot_coords[1]:.3f}\n"
              f"M3: {self._mot_coords[2]:.3f}\n"
              f"M4: {self._mot_coords[3]:.3f}\n"
              f"M5: {self._mot_coords[4]:.3f}\n"
              f"M6: {self._mot_coords[5]:.3f}\n")

        print(f"Joint coordinates:\n"
              f"J1: {self._jnt_coords[0]:.3f}\n"
              f"J2: {self._jnt_coords[1]:.3f}\n"
              f"J3: {self._jnt_coords[2]:.3f}\n"
              f"J4: {self._jnt_coords[3]:.3f}\n"
              f"J5: {self._jnt_coords[4]:.3f}\n"
              f"J6: {self._jnt_coords[5]:.3f}\n")

        print(f"Base jacobian:\n{np.round(self._base_jacobian, 3)}\n")
        print(f"Tool jacobian:\n{np.round(self._tool_jacobian, 3)}\n")

    def _f_kin(self, jnt_coords: np.ndarray) -> Pose:
        """ Forward kinematics. Computes OPS coords [X Y Z W I J K] from given joint 8-vector.
        Source: L.Sciavicco and B.Siciliano, Modelling and Control of Robot Manipulators
        @param jnt_coords: np.array, 6-vector of joint coordinates
        @return: np.array of OPS-coords
        """
        # DH-parameters, manipulator dimension constants
        a1, a2, a3 = DH_PARAMS['a1'], DH_PARAMS['a2'], DH_PARAMS['a3']
        d1, d4, d6 = DH_PARAMS['d1'], DH_PARAMS['d4'], DH_PARAMS['d6']

        # Joint values
        th1 = jnt_coords[0]
        th2 = jnt_coords[1]
        th3 = jnt_coords[2]
        th4 = jnt_coords[3]
        th5 = jnt_coords[4]
        th6 = jnt_coords[5]

        # Account for custom zero-position
        th2 += math.pi / 2

        # Precomputing trig functions
        c1, c2, c4, c5, c6, c23 = cos(th1), cos(th2), cos(th4), cos(th5), cos(th6), cos(th2 + th3)
        s1, s2, s4, s5, s6, s23 = sin(th1), sin(th2), sin(th4), sin(th5), sin(th6), sin(th2 + th3)

        # Precompute repeating expressions
        # i and j components of tool frames x-unit vector
        x_hat_i = c23*(c4*c5*c6 - s4*s6) - s23*s5*c6
        x_hat_j = s4*c5*c6 + c4*s6
        # i and j components of tool frames y-unit vector
        y_hat_i = -c23*(c4*c5*s6 + s4*c6) + s23*s5*s6
        y_hat_j = -s4*c5*s6 + c4*c6

        # Wrist pose respect to base frame
        wrist_pose = np.eye(4)

        # Spherical wrist position in base frame
        wrist_pose[0, 3] = a1*c1 + a2*c1*c2 + a3*c1*c23 + d4*c1*s23
        wrist_pose[1, 3] = a1*s1 + a2*s1*c2 + a3*s1*c23 + d4*s1*s23
        wrist_pose[2, 3] = d1 + a2*s2 + a3*s23 - d4*c23

        # Wrist orientation respect to base frame (Tool orientation to be added)
        wrist_pose[0, 0] = c1*x_hat_i + s1*x_hat_j
        wrist_pose[1, 0] = s1*x_hat_i - c1*x_hat_j
        wrist_pose[2, 0] = s23*(c4*c5*c6 - s4*s6) + c23*s5*c6
        wrist_pose[0, 1] = c1*y_hat_i + s1*y_hat_j
        wrist_pose[1, 1] = s1*y_hat_i - c1*y_hat_j
        wrist_pose[2, 1] = -s23*(c4*c5*s6 + s4*c6) - c23*s5*s6
        wrist_pose[:3, 2] = np.cross(wrist_pose[:3, 0], wrist_pose[:3, 1])

        # Tool frame respect to base frame
        tool_pose = wrist_pose @ config.TOOL_OFS

        return Pose.from_SE3(tool_pose)

    def _i_kin(self, target_pose: Pose, prev_jnt_vec: np.ndarray, shoulder_flip: bool = False,
               elbow_down: bool = False, wrist_flip: bool = False) -> Tuple[Union[None, np.ndarray], int]:
        """ Inverse kinematics. Computes join (1-6) values for given posture.
        Custom implementation of information found in L.Sciavicco and B.Siciliano, Modelling and Control of Robot Manipulators
        @param pose: End posture, [X Y Z W I J K]
        @param shoulder_flip: If False, J2<0 for leaning forward, if True, J2>0 for leaning forward.
        @param elbow_down: False: Elbow angled upwards. True: Elbow angled downwards.
        @param wrist_flip: ?
        Currently not used and the solution with the least motor movement is chosen.
        @return: Tuple[IK_SOLUTION, jnt_coords].
        """
        # print(f"IK: q1: {pose[3]:.3f}\tq2: {pose[4]:.3f}\tq3: {pose[5]:.3f}\tq4: {pose[6]:.3f}")
        # DH-parameters, manipulator dimension constants
        a1, a2, a3 = DH_PARAMS['a1'], DH_PARAMS['a2'], DH_PARAMS['a3']
        d1, d4, d6 = DH_PARAMS['d1'], DH_PARAMS['d4'], DH_PARAMS['d6']

        # Because we have elbow offset(a3), we need the distance and angle
        # of the virtual d4 vector from J3 axis to spherical wrist
        elbow2wrist = np.sqrt(a3 * a3 + d4 * d4)
        th3off = atan(a3 / d4)  # Theta 3 offset

        # Backwards rotation from target pose to find wrist pose
        wrist_pose = target_pose.SE3 @ config.INV_TOOL_OFS
        wx, wy, wz = wrist_pose[0, 3], wrist_pose[1, 3], wrist_pose[2, 3]

        # Solve Joint 1 and check limit
        # TODO: wy = wx = 0 leads to shoulder singularity! Then we must define J1 based on additional information!
        # if wx == 0 and wy == 0:
        #     solution = 10
        #     theta1 = J1_previous

        if shoulder_flip:
            theta1 = pi + math.atan2(wy, wx)
        else:
            theta1 = math.atan2(wy, wx)
            # if max < theta1 < min
        if not JOINT_LIMITS['J1_MIN'] <= np.rad2deg(theta1) <= JOINT_LIMITS['J1_MAX']:
            return None, IK_SOLUTION['J1_LIM_TRIG']

        # Shift spherical wrist location closer to accounting for d1 and a1 offsets
        c1, s1 = cos(theta1), sin(theta1)
        wx -= a1 * c1
        wy -= a1 * s1
        wz -= d1

        # Solve Joint 3 and check limit
        base2wrist_sqr = wx*wx + wy*wy + wz*wz  # Repeating expression
        cos_theta3 = (base2wrist_sqr - a2 * a2 - elbow2wrist * elbow2wrist) / (2.0 * a2 * elbow2wrist)
        # Point out of reach: No solution
        if cos_theta3 < -1 or cos_theta3 > 1:
            return None, IK_SOLUTION['ELBOW_SINGULARITY']
        if elbow_down:
            sin_theta3 = math.sqrt(1 - cos_theta3 * cos_theta3)
        else:
            sin_theta3 = -math.sqrt(1 - cos_theta3 * cos_theta3)
        if shoulder_flip:
            theta3 = -math.atan2(sin_theta3, cos_theta3)
        else:
            theta3 = math.atan2(sin_theta3, cos_theta3)
        theta3 += math.pi/2  # Custom zero offset
        theta3 -= th3off  # Angle offset caused by DH-parameter "a3"
        if not JOINT_LIMITS['J3_MIN'] <= np.rad2deg(theta3) <= JOINT_LIMITS['J3_MAX']:
            return None, IK_SOLUTION['J3_LIM_TRIG']

        # Solve Joint 2 and check limit
        cos_theta2 = ((a2 + elbow2wrist * cos_theta3) * math.sqrt(wx * wx + wy * wy) + elbow2wrist * sin_theta3 * wz) / base2wrist_sqr
        sin_theta2 = ((a2 + elbow2wrist * cos_theta3) * wz - elbow2wrist * sin_theta3 * math.sqrt(wx * wx + wy * wy)) / base2wrist_sqr
        if shoulder_flip:
            theta2 = pi - math.atan2(sin_theta2, cos_theta2)
        else:
            theta2 = math.atan2(sin_theta2, cos_theta2)
        theta2 -= math.pi/2  # Custom zero offset
        if not JOINT_LIMITS['J2_MIN'] <= np.rad2deg(theta2) <= JOINT_LIMITS['J2_MAX']:
            # print(f"IK: {theta2}")
            return None, IK_SOLUTION['J2_LIM_TRIG']

        # Spherical wrists: Solve Joints 4-6. Compute orientation resulting from joints 1-3
        # and subtract that from desired end effector orientation
        c23 = np.cos(theta2 + theta3)
        s23 = np.sin(theta2 + theta3)

        # Anthropomorphic arms transposed / inverted orientation
        pose_arm_inverted = np.array([
            [-c1 * s23, -s1 * s23, c23],
            [s1, -c1, 0],
            [c1 * c23, s1 * c23, s23]])
        # print(f"pose_arm_inverted: {pose_arm_inverted}\n")

        # pose_wrist_rot_mat describes desired end effector pose respect to arms (link 3) current pose
        pose_wrist_rot_mat = pose_arm_inverted @ wrist_pose[:3, :3]
        # print(f"Pose wrist rot: {pose_wrist_rot_mat}")

        # Convert to ZYZ Euler angles and checking limits
        wrist_sol_1, wrist_singularity_1 = utils.rot2zyz(pose_wrist_rot_mat, phi_prev=prev_jnt_vec[3], psi_prev=prev_jnt_vec[5], flip=True)
        wrist_sol_2, wrist_singularity_2 = utils.rot2zyz(pose_wrist_rot_mat, phi_prev=prev_jnt_vec[3], psi_prev=prev_jnt_vec[5], flip=False)

        # Check wrist joints solutions and prefer the solution with less distance in joint space.
        wrist_solutions = [wrist_sol_1, wrist_sol_2]
        wrist_solutions.sort(key=lambda x: LA.norm(x-prev_jnt_vec[3:6]))
        error = None
        for sol in wrist_solutions:
            theta4 = sol[0]
            if not JOINT_LIMITS['J4_MIN'] <= np.rad2deg(theta4) <= JOINT_LIMITS['J4_MAX']:
                error = IK_SOLUTION['J4_LIM_TRIG']
                continue
            theta5 = sol[1]
            if not JOINT_LIMITS['J5_MIN'] <= np.rad2deg(theta5) <= JOINT_LIMITS['J5_MAX']:
                error = IK_SOLUTION['J5_LIM_TRIG']
                continue
            theta6 = sol[2]
            if not JOINT_LIMITS['J6_MIN'] <= np.rad2deg(theta6) <= JOINT_LIMITS['J6_MAX']:
                error = IK_SOLUTION['J6_LIM_TRIG']
                continue
            if error is None:
                break

        if error is not None:
            return None, error

        return np.array((theta1, theta2, theta3, theta4, theta5, theta6)), IK_SOLUTION['SUCCESS']

    def _compute_base_jacobian(self, jnt_vec: np.ndarray) -> np.ndarray:
        """ Computes the 6x6 Jacobian matrix between motor space and operational space.
        Source: L.Sciavicco and B.Siciliano, Modelling and Control of Robot Manipulators for
        diagonal blocks and own derivations for off-diagonal blocks.
        Reads and writes to/from internal state.
        """

        # DH-parameters, manipulator dimension constants
        a1 = DH_PARAMS['a1']
        a2 = DH_PARAMS['a2']
        a3 = DH_PARAMS['a3']
        d4 = DH_PARAMS['d4']
        d6 = DH_PARAMS['d6']

        # Current joint coordinates
        th1 = jnt_vec[0]
        th2 = jnt_vec[1] + math.pi / 2
        th3 = jnt_vec[2]
        th4 = jnt_vec[3]
        th5 = jnt_vec[4]

        # Precomputing trig functions
        c1, c2, c3, c4, c5, c23 = cos(th1), cos(th2), cos(th3), cos(th4), cos(th5), cos(th2 + th3)
        s1, s2, s3, s4, s5, s23 = sin(th1), sin(th2), sin(th3), sin(th4), sin(th5), sin(th2 + th3)

        J = np.zeros((6, N_REV_JNT))

        # Upper left - Joint 1-3 contribution to X Y Z
        # Joint 1 contribution to X Y Z
        J[0][0] = -d6*(s1*(s23*c5 + c23*c4*s5) - c1*s4*s5) - a1*s1 - a3*c23*s1 - d4*s23*s1 - a2*c2*s1
        J[1][0] = d6*(c1*(s23*c5 + c23*c4*s5) + s1*s4*s5) + a1*c1 + a3*c23*c1 + d4*s23*c1 + a2*c1*c2
        J[2][0] = 0.

        # Joint 2 contribution to X Y Z
        dx_djnt2: float = -c1*(a3*s23 - d4*c23 + a2*s2 - d6*c23*c5 + d6*s23*c4*s5)
        dy_djnt2: float = -s1*(a3*s23 - d4*c23 + a2*s2 - d6*c23*c5 + d6*s23*c4*s5)
        dz_djnt2: float = a3*c23 + d4*s23 + a2*c2 + d6*(s23*c5 + c23*c4*s5)
        # Delete. This is for joint space to operational space jacobian.
        # self._base_jacobian[0][1] = -c1*(a3*s23 - d4*c23 + a2*s2 - d6*c23*c5 + d6*s23*c4*s5)
        # self._base_jacobian[1][1] = -s1*(a3*s23 - d4*c23 + a2*s2 - d6*c23*c5 + d6*s23*c4*s5)
        # self._base_jacobian[2][1] = a3*c23 + d4*s23 + a2*c2 + d6*(s23*c5 + c23*c4*s5)

        # Joint 3 contribution to X Y Z
        dx_djnt3 = c1*(d4*c23 - a3*s23 + d6*c23*c5 - d6*s23*c4*s5)
        dy_djnt3 = s1*(d4*c23 - a3*s23 + d6*c23*c5 - d6*s23*c4*s5)
        dz_djnt3 = a3*c23 + d4*s23 + d6*(s23*c5 + c23*c4*s5)
        # Delete. This is for joint space to operational space jacobian.
        # self._base_jacobian[0][2] = c1*(d4*c23 - a3*s23 + d6*c23*c5 - d6*s23*c4*s5)
        # self._base_jacobian[1][2] = s1*(d4*c23 - a3*s23 + d6*c23*c5 - d6*s23*c4*s5)
        # self._base_jacobian[2][2] = a3*c23 + d4*s23 + d6*(s23*c5 + c23*c4*s5)

        # Motor 2 contribution to X Y Z
        J[0][1] = dx_djnt2 - dx_djnt3
        J[1][1] = dy_djnt2 - dy_djnt3
        J[2][1] = dz_djnt2 - dz_djnt3

        # Motor 3 contribution to X Y Z
        J[0][2] = dx_djnt3
        J[1][2] = dy_djnt3
        J[2][2] = dz_djnt3

        # Upper right - Joint 4-6 contribution to X Y Z
        # Joint 4 contribution to X Y Z
        J[0][3] = d6*s5*(c4*s1 - c23*c1*s4)
        J[1][3] = -d6*s5*(c1*c4 + c23*s1*s4)
        J[2][3] = -d6*s23*s4*s5
        # Joint 5 contribution to X Y Z
        J[0][4] = -d6*(c1*(s23*s5 - c23*c4*c5) - c5*s1*s4)
        J[1][4] = -d6*(s1*(s23*s5 - c23*c4*c5) + c1*c5*s4)
        J[2][4] = d6*(c23*s5 + s23*c4*c5)
        # Joint 6 contribution to X Y Z
        J[0][5] = 0.
        J[1][5] = 0.
        J[2][5] = 0.

        # Lower left - Joint 1-3 contribution to RX RY RZ
        # Joint 1 contribution to RX RY RZ
        J[3][0] = 0.
        J[4][0] = 0.
        J[5][0] = 1.
        # Joint 2 contribution to RX RY RZ
        J[3][1] = s1
        J[4][1] = -c1
        J[5][1] = 0.
        # Joint 3 contribution to RX RY RZ
        J[3][2] = s1
        J[4][2] = -c1
        J[5][2] = 0.

        # Lower right - Joint 4-6 contribution to RX RY RZ
        # Joint 4 contribution to RX RY RZ
        J[3][3] = s23*c1
        J[4][3] = s23*s1
        J[5][3] = -c23
        # Joint 5 contribution to RX RY RZ
        J[3][4] = c4*s1 - s4*(c1*c2*c3 - c1*s2*s3)
        J[4][4] = s4*(s1*s2*s3 - c2*c3*s1) - c1*c4
        J[5][4] = -s23*s4
        # Joint 6 contribution to RX RY RZ
        J[3][5] = s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3)) + c5*(c1*c2*s3 + c1*c3*s2)
        J[4][5] = c5*(c2*s1*s3 + c3*s1*s2) - s5*(c1*s4 + c4*(s1*s2*s3 - c2*c3*s1))
        J[5][5] = s23*c4*s5 - c23*c5

        return J

    def _compute_tool_jacobian(self, J: np.ndarray, ops_vec: Pose):
        """ Rotate manipulator jacobian into tool frame """
        tool_rot = utils.quat2rot_mat(ops_vec.quaternion)

        # Block-diagonal rotation matrix
        R = np.eye(6)
        R[:3, :3] = tool_rot
        R[3:, 3:] = tool_rot

        return R @ J  # Jacobian matrix in tool frame

    def _compute_singular_vals_vecs(self):
        """ Compute left singular vectors (U-matrix, operational / output space) and
        corresponding singular values of the manipulator jacobian.
        """
        jacobian_trans = self._base_jacobian[:3, :3]
        jacobian_rot = self._base_jacobian[3:, 3:6]
        # TODO: Include translational axis

        eig_vals_trans, self._sing_vecs_trans = LA.eigh(jacobian_trans @ jacobian_trans.T)
        eig_vals_rot, self._sing_vecs_rot = LA.eigh(jacobian_rot @ jacobian_rot.T)
        self._sing_vals_trans = np.sqrt(abs(eig_vals_trans))
        self._sing_vals_rot = np.sqrt(abs(eig_vals_rot))

    def _compute_condition(self, frame: np.ndarray) -> np.ndarray:
        """
        @param frame:
        @return: 6-array of condition [x, y, z, rx, ry, rz]
        """
        jacobian_trans = self._base_jacobian[:3, :3]
        jacobian_rot = self._base_jacobian[3:, 3:6]
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

    def _move_mot(self, mot_vec: np.ndarray, incremental: bool=False) -> np.ndarray | None:
        """ Execute movement in motor space
        @param mot_vec: 6-vector of motor coordinates. Unit: rad and meters
        @param incremental: Bool, True for incremental motor coordinates, False for absolute.
        @return: True if generation of motion successful and G-code string, False and empty string otherwise.
        """

        # Get absolute motor position as radians and meters
        # mot_pos_rad_meter_abs = mot_vec.copy()
        # if incremental:
        #     mot_pos_rad_meter_abs += self._temp_mot_coords
        #     mot_diff_rad_meter =  mot_vec
        # else:
        #     mot_diff_rad_meter = mot_pos_rad_meter_abs - self._temp_mot_coords
        mot_abs_pos_rad = mot_vec.copy()

        # Check limits
        for idx in range(N_REV_JNT):
            if not MOTOR_LIMITS[f"M{idx+1}_MIN"] <= mot_vec[idx] <= MOTOR_LIMITS[f"M{idx+1}_MAX"]:
                print(f"Revolute motor value out of range")
                return None

        # Update temporary system
        self._update_temp_state(mot_abs_pos_rad)

        return mot_abs_pos_rad

    def _move_jnt(self, jnt_vec: np.ndarray, incremental: bool = False) -> np.ndarray | None:
        """ Execute movement in joint space
        @param jnt_vec: 6-vector of joint coordinates. Unit: rad and meters
        @param incremental: Bool, True for incremental motor coordinates, False for absolute.
        @return: True if generation of motion successful and G-code string, False and empty string otherwise.
        """
        jnt_vec_abs = jnt_vec.copy()
        # # Convert to absolute
        # if incremental:
        #     jnt_vec_abs += self._jnt_coords

        # Check limits
        angle_deg = np.rad2deg(jnt_vec_abs)  # deg
        for idx in range(N_REV_JNT):
            if not JOINT_LIMITS[f"J{idx+1}_MIN"] <= angle_deg[idx] <= JOINT_LIMITS[f"J{idx+1}_MAX"]:
                print(f"Revolute joint out of range")
                return None

        # Compute motor coordinates and propagate motion command downwards
        return self._move_mot(self.jnt2mot(jnt_vec_abs))

    def _update_temp_state(self, mot_vec: np.ndarray) -> None:
        self._temp_mot_coords = mot_vec.copy()
        self._temp_jnt_coords = self.mot2jnt(mot_vec)
        # TODO: Check parameters
        self._temp_ops_coords = self._f_kin(self._temp_jnt_coords)
        self._temp_base_jacobian = self._compute_base_jacobian(self._temp_jnt_coords)
        self._temp_tool_jacobian = self._compute_tool_jacobian(self._temp_base_jacobian, self._temp_ops_coords)
        return None

    # ---------------------------------------------------------------------------------------------
    # ------------------------------------ Public interface ---------------------------------------
    # ---------------------------------------------------------------------------------------------
    @property
    def mot_coords(self):
        return self._mot_coords.copy()

    @property
    def temp_mot_coords(self):
        return self._temp_mot_coords.copy()

    @property
    def jnt_coords(self):
        return self._jnt_coords.copy()

    @property
    def temp_jnt_coords(self):
        return self._temp_jnt_coords.copy()

    @property
    def ops_coords(self):
        return self._ops_coords.copy()

    @property
    def temp_ops_coords(self):
        return self._temp_ops_coords.copy()

    @property
    def singular_vals_vecs_trans(self):
        return self._sing_vals_trans, self._sing_vecs_trans

    @property
    def singular_vals_vecs_rot(self):
        return self._sing_vals_rot, self._sing_vecs_rot

    def update_state(self, mot_vec: np.ndarray):
        """ Update manipulators internal state based on motor value vector parsed from GRBL status
        @param mot_vec: np.array, vector of motor values in radians
        """
        self._mot_coords = mot_vec.copy()
        self._jnt_coords = self.mot2jnt(self._mot_coords)

        # Update internal OPS coords
        self._ops_coords = self._f_kin(self.jnt_coords)
        self._base_jacobian = self._compute_base_jacobian(self._jnt_coords)
        # self._tool_jacobian = self._compute_tool_jacobian(self._jnt_coords)
        self._compute_singular_vals_vecs()

        # Update temp state
        # TODO: Do this?
        self._update_temp_state(self._mot_coords)

    def reset(self):
        self.update_state(np.zeros(6))

    def mot2jnt(self, mot_vec: np.ndarray) -> np.ndarray:
        """ Forward kinematics: motor space -> joint space """
        jnt_vec = mot_vec.copy()
        jnt_vec[2] = mot_vec[2] - mot_vec[1]
        return jnt_vec

    def jnt2mot(self, jnt_vec: np.ndarray) -> np.ndarray:
        """ Inverse kinematics: joint space -> motor space """
        mot_vec = jnt_vec.copy()
        mot_vec[2] = jnt_vec[2] + jnt_vec[1]
        return mot_vec

    def f_kin(self, jnt_coords) -> Pose:
        """ Forward kinematics. See _f_kin for details. """
        return self._f_kin(jnt_coords)

    def i_kin(self, pose: Pose, prev_jnt_vec: np.ndarray, shoulder_flip=False,
              elbow_down=False, wrist_flip=False) -> Tuple[Union[None, np.ndarray], int]:
        """ Inverse kinematics. See _i_kin for details. """
        return self._i_kin(pose, prev_jnt_vec, shoulder_flip, elbow_down, wrist_flip)

    def move_mot(self, mot_vec: np.ndarray, degrees: bool = False, incremental: bool = False) -> np.ndarray | None:
        mot_pos_absolute = mot_vec.copy()
        if degrees:
            mot_pos_absolute = np.deg2rad(mot_pos_absolute)
        return self._move_mot(mot_pos_absolute)

    def move_jnt(self, jnt_vec: np.ndarray, degrees: bool = False, incremental: bool = False) -> np.ndarray | None:
        jnt_pos_absolute = jnt_vec.copy()
        if degrees:
            jnt_pos_absolute = np.deg2rad(jnt_pos_absolute)
        return self._move_jnt(jnt_pos_absolute)

    def move_ops(self, target: Pose, incremental: bool = False,
                 shoulder_flip: bool = False, elbow_down: bool = False, wrist_flip: bool = False) -> np.ndarray | None:
        """ Motor space interpolated motion to given posture.
        :param target:
        :param incremental: Posture as absolute or incremental to current posture
        :param elbow_down: Bool, for i_kin to choose a specific solution
        :param shoulder_flip: Bool, for i_kin to choose a specific solution
        :param wrist_flip: Bool, for i_kin to choose a specific solution
        :return: True if motion was executed
        """

        # Move too short
        if self.ops_coords.is_close(target):
            return None

        target_jnt_vec, ik_sol = self._i_kin(target, prev_jnt_vec=self._temp_jnt_coords.copy(), shoulder_flip=shoulder_flip, elbow_down=elbow_down, wrist_flip=wrist_flip)

        if ik_sol != IK_SOLUTION["SUCCESS"]:
            print(f"move_ops: IK fail: {ik_sol}")
            return None

        # Propagate motion request forwards
        return self._move_jnt(target_jnt_vec)

    def move_ops_lin(self, target_pose: Pose, speed_linear: float = None, speed_angular: float = None,
                     segment_length_m: float = 0.001, segment_size_rad: float = 0.0035, incremental: bool = False) -> Tuple[np.ndarray, float] | None:
        """ Linear move in operational space  coordinates. Splits move into segments size of segment_length
        and computes inverse kinematics for all points. If a  move includes both  rotation and translation,
        :param target_pose:
        :param speed_linear: m/s, linear speed. By default, this is used.
        :param speed_angular: rad/s, rotational speed. Used if no linear speed is given.
        :param segment_length_m: m, Length of translational segment.
        :param segment_size_rad: rad, Size of rotational segment.
        :param incremental: Bool, incremental or absolute move
        :return: True if move was executed
        """
        # Target posture
        # target = pose.copy()
        # if incremental:
        #     target[:3] += self._ops_coords[:3]
        #     target[3:] = utils.quat_multiply(target[3:], self._ops_coords[3:])

        current_pose: Pose = self.ops_coords

        # Translational error, meters
        tool_translation_dist = current_pose.distance(target_pose)
        n_segments_lin = int(np.ceil(tool_translation_dist / segment_length_m))

        # Rotational error, radians
        tool_rotation_dist = target_pose.angle(self._ops_coords)
        n_segments_ang = int(np.ceil(tool_rotation_dist / segment_size_rad))

        # Choose whether rotation or translation determines segment count
        n_segments = max(n_segments_lin, n_segments_ang)
        if n_segments < 1:
            raise ValueError("Move results in zero segments")

        # Compute movement time
        move_time = 0.
        if speed_linear is not None:
            move_time = max(tool_translation_dist / speed_linear, move_time)  # m / (m/s) = s
        if speed_angular is not None:
            move_time = max(tool_rotation_dist / speed_angular, move_time)  # rad / (rad/s) = s
        if move_time == 0:
            raise ValueError("No speed given")

        segment_time = move_time / n_segments  # Seconds

        # Interpolate poses
        poses = utils.pose_interpolator(current_pose, target_pose, segment_count=n_segments)
        if poses is None:
            return None

        mot_vecs = np.zeros((n_segments, N_REV_JNT))
        for idx, pose in enumerate(poses):
            interp_jnt_vec, ik_sol = self.i_kin(pose, prev_jnt_vec=self.temp_jnt_coords)
            if ik_sol != IK_SOLUTION['SUCCESS']:
                print(f"move_ops_lin: IK fail: {ik_sol}")
                return None

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

        end_pose: Pose = self.ops_coords

        # Convert direction vector relative to World/Base frame
        if frame == "Base" or frame == "World":
            pass
        elif frame == "Tool":
            unit_vec = end_pose.rot_mat @ unit_vec
        else:
            raise ValueError("Unknown frame type")

        # Compute end posture
        end_pose.position += distance * unit_vec

        # Compute G-code list for end and intermediate postures
        return self.move_ops_lin(end_pose, speed_linear=speed, incremental=False)

    def rotate_tool(self, direction_vec: tuple[int, int, int], angle: float, speed: float, frame: str) -> np.ndarray | None:
        """ Creates a pure rotation around any axis in any frame.
        :param direction_vec: Rotation axis, any length
        :param angle: Rotation angle, radians by default
        :param speed: Rotation speed, radians/second
        :param frame: Frame respect to which direction vector is described. "World", "Base" or "Tool"
        :param degrees: Unit of angle
        """
        # Normalize direction vector
        direction_vec = np.array(direction_vec)
        unit_vec = direction_vec / LA.norm(direction_vec)

        # Convert direction vector and angle to a quaternion
        quat_rot = utils.dir_vec_angle2quat(unit_vec, angle)  # New orientation w.rot.t current orientation cur_Q_new

        # End posture by rotating current posture
        end_pose: Pose = self.ops_coords
        if frame == "Base" or frame == "World":
            # new orientation w.rot.t. world (base)
            end_pose.quaternion = utils.quat_multiply(quat_rot, end_pose.quaternion)
        elif frame == "Tool":
            end_pose.quaternion = utils.quat_multiply(end_pose.quaternion, quat_rot)
        else:
            raise ValueError("Invalid frame")

        # Compute G-code list for end and intermediate postures
        return self.move_ops_lin(end_pose, speed_angular=speed)
