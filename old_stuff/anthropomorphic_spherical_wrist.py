""" Kinematics model for serial manipulator with anthropomorphic arm and spherical wrist.
Based on the book: L.Sciavicco and B.Siciliano, Modelling and Control of Robot Manipulators.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

import config
from config import *
import utils
from robot_math.pose import Pose

from typing import Tuple, Union
import numpy as np

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


class ASWKinematics:

    def __init__(self, DH: dict):

        self.DH = DH  # Denavit Hartenberg parameters

    @staticmethod
    def mot2jnt(mot_vec: np.ndarray) -> np.ndarray:
        """ Forward kinematics: motor space -> joint space
        :param mot_vec: Absolute motor coordinates, radians.
        :return jnt_vec: Absolute joint coordinates, radians.
        """
        jnt_vec = mot_vec.copy()
        jnt_vec[2] = mot_vec[2] - mot_vec[1]
        return jnt_vec

    @staticmethod
    def jnt2mot(jnt_vec: np.ndarray) -> np.ndarray:
        """ Inverse kinematics: joint space -> motor space
        :param jnt_vec: Absolute joint coordinates, radians.
        :return mot_vec: Absolute motor coordinates, radians.
        """
        mot_vec = jnt_vec.copy()
        mot_vec[2] = jnt_vec[2] + jnt_vec[1]
        return mot_vec

    def forward(self, jnt_vec: np.ndarray):
        """ Forward kinematics. Joint space -> operational space.
        :param jnt_vec: Absolute joint coordinates, radians.
        :return: Pose-object
        """
        # DH-parameters, manipulator dimension constants
        a1, a2, a3 = self.DH['a1'], self.DH['a2'], self.DH['a3']
        d1, d4, d6 = self.DH['d1'], self.DH['d4'], self.DH['d6']

        # Joint values
        th1, th2, th3 = jnt_vec[0], jnt_vec[1], jnt_vec[2]
        th4, th5, th6 = jnt_vec[3], jnt_vec[4], jnt_vec[5]

        # Account for custom zero-position
        th2 += np.pi / 2

        # Precomputing trig functions
        c1, c2, c4, c5, c6, c23 = np.cos(th1), np.cos(th2), np.cos(th4), np.cos(th5), np.cos(th6), np.cos(th2 + th3)
        s1, s2, s4, s5, s6, s23 = np.sin(th1), np.sin(th2), np.sin(th4), np.sin(th5), np.sin(th6), np.sin(th2 + th3)

        # Precompute repeating expressions
        # i and j components of tool frames x-unit vector
        x_hat_i = c23 * (c4 * c5 * c6 - s4 * s6) - s23 * s5 * c6
        x_hat_j = s4 * c5 * c6 + c4 * s6
        # i and j components of tool frames y-unit vector
        y_hat_i = -c23 * (c4 * c5 * s6 + s4 * c6) + s23 * s5 * s6
        y_hat_j = -s4 * c5 * s6 + c4 * c6

        # Wrist pose respect to base frame
        wrist_pose = np.eye(4, dtype=np.float64)

        # Spherical wrist position in base frame
        wrist_pose[0, 3] = a1 * c1 + a2 * c1 * c2 + a3 * c1 * c23 + d4 * c1 * s23
        wrist_pose[1, 3] = a1 * s1 + a2 * s1 * c2 + a3 * s1 * c23 + d4 * s1 * s23
        wrist_pose[2, 3] = d1 + a2 * s2 + a3 * s23 - d4 * c23

        # Wrist orientation respect to base frame (Tool orientation to be added)
        wrist_pose[0, 0] = c1 * x_hat_i + s1 * x_hat_j
        wrist_pose[1, 0] = s1 * x_hat_i - c1 * x_hat_j
        wrist_pose[2, 0] = s23 * (c4 * c5 * c6 - s4 * s6) + c23 * s5 * c6
        wrist_pose[0, 1] = c1 * y_hat_i + s1 * y_hat_j
        wrist_pose[1, 1] = s1 * y_hat_i - c1 * y_hat_j
        wrist_pose[2, 1] = -s23 * (c4 * c5 * s6 + s4 * c6) - c23 * s5 * s6
        wrist_pose[:3, 2] = np.cross(wrist_pose[:3, 0], wrist_pose[:3, 1])

        # Tool frame respect to base frame
        tool_pose = wrist_pose @ config.TOOL_OFS

        return Pose.from_SE3(tool_pose)

    def inverse(self, target_pose: Pose, prev_jnt_vec: np.ndarray, shoulder_flip: bool = False,
               elbow_down: bool = False, wrist_flip: bool = False) -> Tuple[Union[None, np.ndarray], int]:
        """ Inverse kinematics. Operational space -> joint space, if solution exists.
        @param target_pose: 6D Pose object
        @param shoulder_flip: False: J2<0 for leaning forward. True: J2>0 for leaning forward.
        @param elbow_down: False: Elbow angled upwards. True: Elbow angled downwards.
        @param wrist_flip: J5 < 0 or J5 > 0. Currently, not in use.
        @return: Tuple[IK_SOLUTION, jnt_coords].
        """
        # DH-parameters, manipulator dimension constants
        a1, a2, a3 = self.DH['a1'], self.DH['a2'], self.DH['a3']
        d1, d4, d6 = self.DH['d1'], self.DH['d4'], self.DH['d6']

        # Because we have elbow offset(a3), we need the distance and angle
        # of the virtual d4 vector from J3 axis to spherical wrist
        elbow2wrist = np.sqrt(a3 * a3 + d4 * d4)
        th3off = np.atan(a3 / d4)  # Theta 3 offset

        # Backwards rotation from target pose to find wrist pose
        wrist_pose = target_pose.SE3 @ config.INV_TOOL_OFS
        wx, wy, wz = wrist_pose[0, 3], wrist_pose[1, 3], wrist_pose[2, 3]

        # Solve Joint 1 and check limit
        # TODO: wy = wx = 0 leads to shoulder singularity! Then we must define J1 based on additional information!

        if shoulder_flip:
            theta1 = np.pi + np.atan2(wy, wx)
        else:
            theta1 = np.atan2(wy, wx)
            # if max < theta1 < min
        if not JOINT_LIMITS['J1_MIN'] <= np.rad2deg(theta1) <= JOINT_LIMITS['J1_MAX']:
            return None, IK_SOLUTION['J1_LIM_TRIG']

        # Shift spherical wrist location closer to accounting for d1 and a1 offsets
        c1, s1 = np.cos(theta1), np.sin(theta1)
        wx -= a1 * c1
        wy -= a1 * s1
        wz -= d1

        # Solve Joint 3 and check limit
        base2wrist_sqr = wx*wx + wy*wy + wz*wz  # Repeating expression
        cos_theta3 = (base2wrist_sqr - a2 * a2 - elbow2wrist * elbow2wrist) / (2.0 * a2 * elbow2wrist)
        # Point out of reach. No solution.
        if cos_theta3 < -1 or cos_theta3 > 1:
            return None, IK_SOLUTION['ELBOW_SINGULARITY']
        if elbow_down:
            sin_theta3 = np.sqrt(1 - cos_theta3 * cos_theta3)
        else:
            sin_theta3 = -np.sqrt(1 - cos_theta3 * cos_theta3)
        if shoulder_flip:
            theta3 = -np.atan2(sin_theta3, cos_theta3)
        else:
            theta3 = np.atan2(sin_theta3, cos_theta3)
        theta3 += np.pi/2  # Custom zero offset
        theta3 -= th3off  # Angle offset caused by DH-parameter "a3"
        if not JOINT_LIMITS['J3_MIN'] <= np.rad2deg(theta3) <= JOINT_LIMITS['J3_MAX']:
            return None, IK_SOLUTION['J3_LIM_TRIG']

        # Solve Joint 2 and check limit
        cos_theta2 = ((a2 + elbow2wrist * cos_theta3) * np.sqrt(wx * wx + wy * wy) + elbow2wrist * sin_theta3 * wz) / base2wrist_sqr
        sin_theta2 = ((a2 + elbow2wrist * cos_theta3) * wz - elbow2wrist * sin_theta3 * np.sqrt(wx * wx + wy * wy)) / base2wrist_sqr
        if shoulder_flip:
            theta2 = np.pi - np.atan2(sin_theta2, cos_theta2)
        else:
            theta2 = np.atan2(sin_theta2, cos_theta2)
        theta2 -= np.pi/2  # Custom zero offset
        if not JOINT_LIMITS['J2_MIN'] <= np.rad2deg(theta2) <= JOINT_LIMITS['J2_MAX']:
            return None, IK_SOLUTION['J2_LIM_TRIG']

        # Spherical wrists: Solve Joints 4-6. Compute orientation resulting from joints 1-3
        # and subtract that from desired end effector orientation
        c23 = np.cos(theta2 + theta3)
        s23 = np.sin(theta2 + theta3)

        # Anthropomorphic arms transposed / inverted orientation
        pose_arm_inverted = np.array([
            [-c1 * s23, -s1 * s23, c23],
            [s1, -c1, 0],
            [c1 * c23, s1 * c23, s23]], dtype=np.float64)

        # pose_wrist_rot_mat describes desired end effector pose respect to arms (link 3) current pose
        pose_wrist_rot_mat = pose_arm_inverted @ wrist_pose[:3, :3]

        # Convert to ZYZ Euler angles and checking limits
        wrist_sol_1, wrist_singularity_1 = utils.rot2zyz(pose_wrist_rot_mat, phi_prev=prev_jnt_vec[3], psi_prev=prev_jnt_vec[5], flip=True)
        wrist_sol_2, wrist_singularity_2 = utils.rot2zyz(pose_wrist_rot_mat, phi_prev=prev_jnt_vec[3], psi_prev=prev_jnt_vec[5], flip=False)

        # Check wrist joints solutions and prefer the solution with less distance in joint space.
        wrist_solutions = [wrist_sol_1, wrist_sol_2]
        wrist_solutions.sort(key=lambda x: np.linalg.norm(x-prev_jnt_vec[3:6]))
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

    def jacobian(self, jnt_vec: np.ndarray):
        """ Computes the 6x6 Jacobian matrix between motor space and operational space.
        :param jnt_vec: Absolute joint coordinates in radians.
        """

        # DH-parameters, manipulator dimension constants
        a1 = self.DH['a1']
        a2 = self.DH['a2']
        a3 = self.DH['a3']
        d4 = self.DH['d4']
        d6 = self.DH['d6']

        # Current joint coordinates
        th1 = jnt_vec[0]
        th2 = jnt_vec[1] + np.pi / 2
        th3 = jnt_vec[2]
        th4 = jnt_vec[3]
        th5 = jnt_vec[4]

        # Precomputing trig functions
        c1, c2, c3, c4, c5, c23 = np.cos(th1), np.cos(th2), np.cos(th3), np.cos(th4), np.cos(th5), np.cos(th2+th3)
        s1, s2, s3, s4, s5, s23 = np.sin(th1), np.sin(th2), np.sin(th3), np.sin(th4), np.sin(th5), np.sin(th2+th3)

        J = np.zeros((6, N_REV_JNT), dtype=np.float64)

        # Upper left - Joint 1-3 contribution to X Y Z
        # Joint 1 contribution to X Y Z
        J[0][0] = -d6 * (s1 * (s23 * c5 + c23 * c4 * s5) - c1 * s4 * s5) - a1 * s1 - a3 * c23 * s1 - d4 * s23 * s1 - a2 * c2 * s1
        J[1][0] = d6 * (c1 * (s23 * c5 + c23 * c4 * s5) + s1 * s4 * s5) + a1 * c1 + a3 * c23 * c1 + d4 * s23 * c1 + a2 * c1 * c2
        J[2][0] = 0.

        # Joint 2 contribution to X Y Z
        dx_djnt2 = -c1 * (a3 * s23 - d4 * c23 + a2 * s2 - d6 * c23 * c5 + d6 * s23 * c4 * s5)
        dy_djnt2 = -s1 * (a3 * s23 - d4 * c23 + a2 * s2 - d6 * c23 * c5 + d6 * s23 * c4 * s5)
        dz_djnt2 = a3 * c23 + d4 * s23 + a2 * c2 + d6 * (s23 * c5 + c23 * c4 * s5)
        # Delete. This is for joint space to operational space jacobian.
        # self._base_jacobian[0][1] = -c1*(a3*s23 - d4*c23 + a2*s2 - d6*c23*c5 + d6*s23*c4*s5)
        # self._base_jacobian[1][1] = -s1*(a3*s23 - d4*c23 + a2*s2 - d6*c23*c5 + d6*s23*c4*s5)
        # self._base_jacobian[2][1] = a3*c23 + d4*s23 + a2*c2 + d6*(s23*c5 + c23*c4*s5)

        # Joint 3 contribution to X Y Z
        dx_djnt3 = c1 * (d4 * c23 - a3 * s23 + d6 * c23 * c5 - d6 * s23 * c4 * s5)
        dy_djnt3 = s1 * (d4 * c23 - a3 * s23 + d6 * c23 * c5 - d6 * s23 * c4 * s5)
        dz_djnt3 = a3 * c23 + d4 * s23 + d6 * (s23 * c5 + c23 * c4 * s5)
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
        J[0][3] = d6 * s5 * (c4 * s1 - c23 * c1 * s4)
        J[1][3] = -d6 * s5 * (c1 * c4 + c23 * s1 * s4)
        J[2][3] = -d6 * s23 * s4 * s5
        # Joint 5 contribution to X Y Z
        J[0][4] = -d6 * (c1 * (s23 * s5 - c23 * c4 * c5) - c5 * s1 * s4)
        J[1][4] = -d6 * (s1 * (s23 * s5 - c23 * c4 * c5) + c1 * c5 * s4)
        J[2][4] = d6 * (c23 * s5 + s23 * c4 * c5)
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
        J[3][3] = s23 * c1
        J[4][3] = s23 * s1
        J[5][3] = -c23
        # Joint 5 contribution to RX RY RZ
        J[3][4] = c4 * s1 - s4 * (c1 * c2 * c3 - c1 * s2 * s3)
        J[4][4] = s4 * (s1 * s2 * s3 - c2 * c3 * s1) - c1 * c4
        J[5][4] = -s23 * s4
        # Joint 6 contribution to RX RY RZ
        J[3][5] = s5 * (s1 * s4 + c4 * (c1 * c2 * c3 - c1 * s2 * s3)) + c5 * (c1 * c2 * s3 + c1 * c3 * s2)
        J[4][5] = c5 * (c2 * s1 * s3 + c3 * s1 * s2) - s5 * (c1 * s4 + c4 * (s1 * s2 * s3 - c2 * c3 * s1))
        J[5][5] = s23 * c4 * s5 - c23 * c5

        return J

    @staticmethod
    def tool_jacobian(J: np.ndarray, tool_rot_mat: np.ndarray):
        """ Rotate manipulator jacobian into tool frame """
        R = np.eye(6, dtype=np.float64)
        R[:3, :3] = tool_rot_mat
        R[3:, 3:] = tool_rot_mat

        return R @ J  # Jacobian matrix in tool frame
