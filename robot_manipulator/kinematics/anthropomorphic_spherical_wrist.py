""" Kinematics model for serial manipulator with anthropomorphic arm and spherical wrist.
Based on the book: L.Sciavicco and B.Siciliano, Modelling and Control of Robot Manipulators.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from dataclasses import dataclass, field
from enum import Enum, auto
from config import *
import config
from robot_math.pose import Pose
from robot_math.zyz_euler import ZYZEuler
from typing import List, Dict


class Singularity(Enum):
    NO_SINGULARITY = auto()
    SHOULDER_SINGULARITY =  auto()
    ELBOW_SINGULARITY =  auto()
    WRIST_SINGULARITY =  auto()
    WRIST_FLIPPED =  auto()

class JointLimit(Enum):
    CLEAR = auto()
    J1_LIMIT = auto()
    J2_LIMIT = auto()
    J3_LIMIT = auto()
    J4_LIMIT = auto()
    J5_LIMIT = auto()
    J6_LIMIT = auto()


@dataclass
class IkSolution:
    singularity: Singularity = Singularity.NO_SINGULARITY
    joint_limits: JointLimit = JointLimit.CLEAR
    joint_solution: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float64))
    success: bool = False


class ASWKinematics:

    def __init__(self, DH: List[dict]):

        # Denavit-Hartenberg table.
        self.DH = DH

        # Trigonometric functions for kinematic calibration.
        self.sin_alpha = []
        self.cos_alpha = []
        for dh_row in self.DH:
            self.sin_alpha.append(np.sin(dh_row['alpha'], dtype=np.float64))
            self.cos_alpha.append(np.cos(dh_row['alpha'], dtype=np.float64))

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

    def forward(self, jnt_vec: np.ndarray) -> Dict[str, List[Pose]]:
        """ Forward kinematics. Joint space -> operational space.
        :param jnt_vec: Absolute joint coordinates, radians.
        :return: List of Pose-objects, one for each link.
        """
        mot_vec = self.jnt2mot(jnt_vec)
        m2: np.float64 = mot_vec[1]
        m3: np.float64 = mot_vec[2]

        link_poses: List[Pose] = []
        T_previous: np.ndarray = np.eye(4, dtype=np.float64)
        for idx, row in enumerate(self.DH):
            # Trig functions
            s_nu, c_nu = np.sin(jnt_vec[idx] + row['nu_offset']), np.cos(jnt_vec[idx] + row['nu_offset'])
            s_al, c_al = self.sin_alpha[idx], self.cos_alpha[idx]

            # Construct SE3 transformation matrix. Link i w.r.t. link i-1.
            T: np.ndarray = np.array([
                [c_nu, -s_nu*c_al, s_nu*s_al, row['a']*c_nu],
                [s_nu, c_nu*c_al, -c_nu*s_al, row['a']*s_nu],
                [np.float64(0), s_al, c_al, row['d']],
                [np.float64(0), np.float64(0), np.float64(0), np.float64(1)]
            ])
            # Append new composite pose representing link i w.r.t. base.
            link_i_wrt_base: np.ndarray = T_previous @ T
            link_poses.append(Pose(SE3=link_i_wrt_base))
            T_previous = link_i_wrt_base

        # Add tool transformation.
        link_poses.append(Pose(SE3=link_poses[-1].SE3 @ config.TOOL_OFS))

        spring_poses = self.spring_poses(T1=link_poses[0], q2=jnt_vec[1])
        counter_weight_pose: Pose = self.counter_weight_pose(T1=link_poses[0], mot3=m3)
        parallel_link_pose: Pose = self.parallel_link_pose(T1=link_poses[0], mot2=m2, mot3=m3)

        return {
            "link_poses": link_poses,
            "spring_poses": spring_poses,
            "counter_weight_pose": counter_weight_pose,
            "parallel_link_pose": parallel_link_pose,
        }

    @staticmethod
    def spring_poses(T1: Pose, q2: np.float64) -> List[Pose]:
        """ Computes poses for link 2 outer counter springs.
        :param T1: Link 1 w.r.t. base.
        :param q2: Joint 2 value in radians.
        :return: Poses top and bottom part for left and right spring.
        """
        L = config.L2_LENGTH
        a = np.float64(L + config.ELBOW_TO_SPRING)
        c = config.SHOULDER_TO_SPRING
        b_sqr = 2*L*(L - c*np.sin(q2) + c**2)  # Counter spring length squared
        gamma = np.acos(a**2 + b_sqr + c**2 / (2*a*np.sqrt(b_sqr)), dtype=np.float64)  # Angle between Link 2 and counter spring

        if q2 < 0:  # Forward
            delta = np.float64(q2 - gamma)
        else:  # Backward
            delta = np.float64(q2 + gamma)

        R = np.eye(3)
        R[0, 0], R[0, 1] = np.cos(delta, dtype=np.float64), -np.sin(delta, dtype=np.float64)
        R[1, 0], R[1, 1] = np.sin(delta, dtype=np.float64), np.cos(delta, dtype=np.float64)

        def spring_transform(xyz: np.ndarray, rot: np.ndarray) -> np.ndarray:
            T = np.eye(4)
            T[:3, 3] = xyz
            T[:3, :3] = rot
            return T

        left_down: Pose = Pose(T1.SE3 @ spring_transform(np.array([0, 0, config.SPRING_CENTER_DIST], dtype=np.float64), rot=R))
        right_down: Pose = Pose(T1.SE3 @ spring_transform(np.array([0, 0, -config.SPRING_CENTER_DIST], dtype=np.float64), rot=R))
        left_up: Pose = Pose(T1.SE3 @ spring_transform(np.array([np.sin(a), np.cos(a), config.SPRING_CENTER_DIST], dtype=np.float64), rot=R))
        right_up: Pose = Pose(T1.SE3 @ spring_transform(np.array([np.sin(a), np.cos(a), -config.SPRING_CENTER_DIST], dtype=np.float64), rot=R))

        return [left_down, right_down, left_up, right_up]

    @staticmethod
    def counter_weight_pose(T1: Pose, mot3: np.float64) -> Pose:
        s_nu, c_nu = np.sin(mot3), np.cos(mot3)
        T = np.eye(4)
        T[0, 0], T[0, 1] = c_nu, -s_nu
        T[1, 0], T[1, 1] = s_nu, c_nu
        return Pose(T1.SE3 @ T)

    @staticmethod
    def parallel_link_pose(T1: Pose, mot2: np.float64, mot3: np.float64):
        parallel_link_pose: np.ndarray = np.eye(4)
        parallel_link_pose[:3, 3] = np.array([-config.PARALLEL_LINK_DIST*np.cos(mot3), -config.PARALLEL_LINK_DIST*np.sin(mot3), 0.], dtype=np.float64)
        s_m2, c_m2 = np.sin(mot2), np.cos(mot2)
        R = np.eye(3)
        R[0, 0], R[0, 1] = c_m2, -s_m2
        R[1, 0], R[1, 1] = s_m2, c_m2
        parallel_link_pose[:3, :3] = R
        return Pose(T1.SE3 @ parallel_link_pose)

    def inverse(self, target_pose: Pose, prev_jnt_vec: np.ndarray, shoulder_flip: bool = False,
               elbow_down: bool = False, wrist_flip: bool = False, jnt_correction=False) -> IkSolution:
        """ Inverse kinematics. Operational space -> joint space, if solution exists.
        :param target_pose: 6D Pose object.
        :param prev_jnt_vec: Previous solution used in case of singularity.
        :param shoulder_flip: False: J2<0 for leaning forward. True: J2>0 for leaning forward.
        :param elbow_down: False: Elbow angled upwards. True: Elbow angled downwards.
        :param wrist_flip: J5 < 0 or J5 > 0. Currently, not in use. Closer solution is chosen.
        :param jnt_correction: Additional correction vector based on kinematic calibration.
        :return: IkSolution.
        """
        # DH-parameters, manipulator dimension constants
        a1, a2, a3 = self.DH[0]['a'], self.DH[1]['a'], self.DH[2]['a']
        d1, d4, d6 = self.DH[0]['d'], self.DH[3]['d'], self.DH[5]['d']
        # print(f"IK: Requested pose: {target_pose}")

        # Because we have elbow offset(a3), we need the distance and angle
        # of the virtual d4 vector from J3 axis to spherical wrist
        elbow2wrist = np.sqrt(a3 * a3 + d4 * d4)
        th3off = np.atan(a3 / d4)  # Theta 3 offset.

        # Backwards rotation from target pose to find tool flange.
        tool_flange_pose = target_pose.SE3 @ config.INV_TOOL_OFS
        # Backwards translation to find spherical wrist pose.
        wrist_pose: np.ndarray = tool_flange_pose.copy()
        wrist_pose[:3, 3] -= d6 * tool_flange_pose[:3, 2]
        wx, wy, wz = wrist_pose[0, 3], wrist_pose[1, 3], wrist_pose[2, 3]

        # Solve Joint 1 and check limit.
        # TODO: wy = wx = 0 leads to shoulder singularity! Then we must define J1 based on additional information!

        if shoulder_flip:
            theta1 = np.pi + np.atan2(wy, wx)
        else:
            theta1 = np.atan2(wy, wx)
        if not JOINT_LIMITS['J1_MIN'] <= np.rad2deg(theta1) <= JOINT_LIMITS['J1_MAX']:
            return IkSolution(joint_limits=JointLimit.J1_LIMIT)

        # Shift spherical wrist location closer to accounting for d1 and a1 offsets.
        c1, s1 = np.cos(theta1), np.sin(theta1)
        wx -= a1 * c1
        wy -= a1 * s1
        wz -= d1

        # Solve Joint 3 and check limit.
        base2wrist_sqr = wx*wx + wy*wy + wz*wz  # Repeating expression.
        cos_theta3 = (base2wrist_sqr - a2 * a2 - elbow2wrist * elbow2wrist) / (2.0 * a2 * elbow2wrist)
        # Point out of reach. No solution.
        if cos_theta3 < -1 or cos_theta3 > 1:
            return IkSolution(singularity = Singularity.ELBOW_SINGULARITY)

        if elbow_down:
            sin_theta3 = np.sqrt(1 - cos_theta3 * cos_theta3)
        else:
            sin_theta3 = -np.sqrt(1 - cos_theta3 * cos_theta3)
        if shoulder_flip:
            theta3 = -np.atan2(sin_theta3, cos_theta3)
        else:
            theta3 = np.atan2(sin_theta3, cos_theta3)
        theta3 += np.pi/2  # Custom zero offset.
        theta3 -= th3off  # Angle offset caused by DH-parameter "a3".
        if not JOINT_LIMITS['J3_MIN'] <= np.rad2deg(theta3) <= JOINT_LIMITS['J3_MAX']:
            return IkSolution(joint_limits=JointLimit.J3_LIMIT)

        # Solve Joint 2 and check limit.
        cos_theta2 = ((a2 + elbow2wrist * cos_theta3) * np.sqrt(wx * wx + wy * wy) + elbow2wrist * sin_theta3 * wz) / base2wrist_sqr
        sin_theta2 = ((a2 + elbow2wrist * cos_theta3) * wz - elbow2wrist * sin_theta3 * np.sqrt(wx * wx + wy * wy)) / base2wrist_sqr
        if shoulder_flip:
            theta2 = np.pi - np.atan2(sin_theta2, cos_theta2)
        else:
            theta2 = np.atan2(sin_theta2, cos_theta2)
        theta2 -= np.pi/2  # Custom zero offset.
        if not JOINT_LIMITS['J2_MIN'] <= np.rad2deg(theta2) <= JOINT_LIMITS['J2_MAX']:
            return IkSolution(joint_limits=JointLimit.J2_LIMIT)

        # Spherical wrists: Solve Joints 4-6. Compute orientation resulting from joints 1-3.
        # and subtract that from desired end effector orientation.
        c23 = np.cos(theta2 + theta3)
        s23 = np.sin(theta2 + theta3)

        # Anthropomorphic arms transposed / inverted orientation.
        pose_arm_inverted = np.array([
            [-c1 * s23, -s1 * s23, c23],
            [s1, -c1, 0],
            [c1 * c23, s1 * c23, s23]], dtype=np.float64)

        # pose_wrist_rot_mat describes desired end effector pose respect to arms (link 3) current pose.
        pose_wrist_rot_mat = pose_arm_inverted @ wrist_pose[:3, :3]

        # Convert to ZYZ Euler angles and checking limits.
        wrist_sol_1: ZYZEuler = ZYZEuler.from_rot_mat(pose_wrist_rot_mat, phi_prev=prev_jnt_vec[3], psi_prev=prev_jnt_vec[5], flip=True)
        wrist_sol_2: ZYZEuler = ZYZEuler.from_rot_mat(pose_wrist_rot_mat, phi_prev=prev_jnt_vec[3], psi_prev=prev_jnt_vec[5], flip=False)

        # Check wrist joints solutions and prefer the solution with less distance in joint space.
        wrist_solutions = [wrist_sol_1, wrist_sol_2]
        wrist_solutions.sort(key=lambda x: np.linalg.norm(x-prev_jnt_vec[3:6]))
        theta4, theta5, theta6 = 0., 0., 0.
        limit_trigger: JointLimit = JointLimit.CLEAR
        for sol in wrist_solutions:
            theta4 = sol[0]
            if not JOINT_LIMITS['J4_MIN'] <= np.rad2deg(theta4) <= JOINT_LIMITS['J4_MAX']:
                limit_trigger = JointLimit.J4_LIMIT
                continue
            theta5 = sol[1]
            if not JOINT_LIMITS['J5_MIN'] <= np.rad2deg(theta5) <= JOINT_LIMITS['J5_MAX']:
                limit_trigger = JointLimit.J5_LIMIT
                continue
            theta6 = sol[2]
            if not JOINT_LIMITS['J6_MIN'] <= np.rad2deg(theta6) <= JOINT_LIMITS['J6_MAX']:
                limit_trigger = JointLimit.J6_LIMIT
                continue
            if limit_trigger is JointLimit.CLEAR:
                break

        # print(f"IK solution: {np.rad2deg(np.array((theta1, theta2, theta3, theta4, theta5, theta6)))} degrees.")
        if limit_trigger != JointLimit.CLEAR:
            return IkSolution(joint_limits=limit_trigger)

        return IkSolution(joint_solution=np.array([theta1, theta2, theta3, theta4, theta5, theta6]), success=True)



    def jacobian(self, jnt_vec: np.ndarray):
        """ Computes the 6x6 Jacobian matrix between motor space and operational space.
        :param jnt_vec: Absolute joint coordinates in radians.
        """

        # DH-parameters, manipulator dimension constants
        a1, a2, a3 = self.DH[0]['a'], self.DH[1]['a'], self.DH[2]['a']
        d1, d4, d6 = self.DH[0]['d'], self.DH[3]['d'], self.DH[5]['d']

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
        """
        :param J: Jacobian respect to base frame.
        :param tool_rot_mat: Base to tool rotation matrix.
        :return: Jacobian respect to tool frame.
        """
        R = np.eye(6, dtype=np.float64)
        R[:3, :3] = tool_rot_mat
        R[3:, 3:] = tool_rot_mat

        return R @ J  # Jacobian matrix in tool frame
    
    def parameter_jacobian(self, jnt_vec: np.ndarray) -> np.ndarray:
        """ Jacobian of cartesian space respect to parameter space.
        :param jnt_vec: Absolute joint coordinates in radians.
        :return:
        """
        # TODO: Consider autodifferentiation?
        # Parameter space
        a1, a2, a3, a4, a5, a6 = self.DH[0]['a'], self.DH[1]['a'], self.DH[2]['a'], self.DH[3]['a'], self.DH[4]['a'], self.DH[5]['a']
        al1, al2, al3, al4, al5, al6 = self.DH[0]['alpha'], self.DH[1]['alpha'], self.DH[2]['alpha'], self.DH[3]['alpha'], self.DH[4]['alpha'], self.DH[5]['alpha']
        d1, d2, d3, d4, d5, d6 = self.DH[0]['d'], self.DH[1]['d'], self.DH[2]['d'], self.DH[3]['d'], self.DH[4]['d'], self.DH[5]['d']
        nu1, nu2, nu3, nu4, nu5, nu6 = self.DH[0]['nu'], self.DH[1]['nu'], self.DH[2]['nu'], self.DH[3]['nu'], self.DH[4]['nu'], self.DH[5]['nu']

        J = np.zeros((6, 24))

        return J

    def get_machine_constants(self):
        # DH-parameters, manipulator dimension constants
        a1, a2, a3 = self.DH[0]['a'], self.DH[1]['a'], self.DH[2]['a']
        d1, d4, d6 = self.DH[0]['d'], self.DH[3]['d'], self.DH[5]['d']
        return a1, a2, a3, d1, d4, d6