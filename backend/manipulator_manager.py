"""
Wrapper class for Manipulator
Author: Rainer Meyer, rot.meyer494@gmail.com
"""

from backend.manipulator import Manipulator
from backend.linear_axis import LinearAxis
from backend.manipulator import IK_SOLUTION
from backend.g_code_writer import GCodeWriter
from backend.pose import Pose
from config import *
from backend import utils
from program.instructions.instruction import Instruction
from program.program import Program
from program.instructions.motion import MoveJ
from program.target import Target

from typing import Tuple
import numpy as np
import numpy.linalg as LA
from PyQt6.QtCore import QObject, pyqtSignal


class ManipulatorManager(QObject):

    g_code_generated = pyqtSignal(str)

    def __init__(self):

        super().__init__()

        self._manipulator = Manipulator()
        self._linear_base = LinearAxis()

        # TODO: Add linear platform class. For now world-to-frame is a constant offset
        # self._T_base_wrt_world = np.eye(4)
        self._T_base_wrt_world = np.array([[0., 1., 0., 0.],
                                           [-1., 0., 0., 0.],
                                           [0., 0., 1., 0.],
                                           [0., 0., 0., 1.]])

        self.gc_writer = GCodeWriter()

        self._T_base_wrt_world[:3, 3] = np.array((0.1, 0.1, 0.))  # Base w.rot.t. World transformation
        self._T_world_wrt_base = np.linalg.inv(self._T_base_wrt_world)  # World w.rot.t. Base transformation
        # Rotation
        self._quat_base_wrt_world = utils.rot_mat2quat(self._T_base_wrt_world[:3, :3])  # Base w.rot.t. World rotation
        self._quat_world_wrt_base =  self._quat_base_wrt_world
        self._quat_world_wrt_base[1:] *= -1  # World w.rot.t. Base rotation


        my_target_1 = Target("T1", pose=np.array([0.280, 0., 0.325, 0., 0.707, 0., 0.707], dtype=float))  # Type: target
        my_move_1 = MoveJ(target=my_target_1)
        self.my_program = Program()
        self.my_program.append(my_move_1)

    # def i_kin_7d(self, pose: np.ndarray, quat_err: np.ndarray, criteria) -> Tuple[np.ndarray | None, IK_SOLUTION]:
    #     """ Numerical inverse kinematics to solve 7 joints for given pose and an additional criteria
    #     @param pose: Position vector and orientation quaternion in world frame
    #     @param criteria: Additional criteria for specific joint solution
    #     """
    #     # TODO: Quat erro in caller function
    #     # quat_err = utils.quat_multiply(quat_desired, quat_current.inv)
    #     # Quaternion error to rotation vector (for small errors only)
    #     rot_vec = 2 * quat_err[1:]  # Small angle approximation
    #     delta_x = np.hstack((pos_err, rot_vec)).T
    #
    #     converge = False
    #     iters = 0
    #     while not converge and iters > 200:
    #         # Fetch jacobian and do pseudo inverse
    #         jacobian = np.hstack((self._manipulator.base_jacobian, self._linear_base.jacobian))  # 6x7
    #         jacobian_pinv = LA.pinv(jacobian)
    #         delta_q = jacobian_pinv @ delta_x
    #
    #
    #
    #
    # def move_ops_lin_7d(self, pose: np.ndarray, speed_linear: float = None, speed_angular: float = None,
    #                  segment_length_m: float = 0.001, segment_size_rad: float = 0.0035, incremental: bool = False) -> bool:
    #     """ Linear move in operational space  coordinates. Splits move into segments size of segment_length
    #     and computes inverse kinematics for all points. If a  move includes both  rotation and translation,
    #     :param pose: 7-np-array, position vector + orientation quaternion [x, y, z, w, i, j, k], world frame
    #     :param speed_linear: m/s, linear speed. By default, this is used.
    #     :param speed_angular: rad/s, rotational speed. Used if no linear speed is given.
    #     :param segment_length_m: m, Length of translational segment.
    #     :param segment_size_rad: rad, Size of rotational segment.
    #     :param incremental: Bool, incremental or absolute move
    #     :return: True if move was executed
    #     """
    #     success = True
    #     g_code_list = []
    #
    #     # Target posture
    #     target = pose.copy()
    #     if incremental:
    #         target[:3] += self._ops_coords[:3]
    #         target[3:] = utils.quat_multiply(target[3:], self._ops_coords[3:])
    #
    #     # Translational error, meters
    #     translation = target[:3] - self._ops_coords[:3]
    #     tool_translation_dist = LA.norm(translation)
    #     n_segments_lin = int(np.ceil(tool_translation_dist / segment_length_m))
    #
    #     # Rotational error, radians
    #     similarity = np.clip(np.dot(self._ops_coords[3:], target[3:]), -1, 1)
    #     if similarity < 0:
    #         target[3:] *= -1
    #         similarity *= -1
    #     tool_rotation_dist = 2*math.acos(similarity)
    #     n_segments_ang = int(np.ceil(tool_rotation_dist / segment_size_rad))
    #
    #     # Choose whether  rotation or translation determines segment count
    #     n_segments = max(n_segments_lin, n_segments_ang)
    #     if n_segments < 1:
    #         raise ValueError("Move results in zero segments")
    #
    #     # Compute movement time
    #     move_time = 0.
    #     if speed_linear is not None:
    #         move_time = max(tool_translation_dist / speed_linear, move_time)  # m / (m/s) = s
    #     if speed_angular is not None:
    #         move_time = max(tool_rotation_dist / speed_angular, move_time)  # rad / (rad/s) = s
    #     if move_time == 0:
    #         raise ValueError("No speed given")
    #
    #     segment_time = move_time / n_segments  # Seconds
    #
    #     # Check if all interpolated points are reachable
    #     previous_jnt_vec = self._jnt_coords.copy()
    #     jnt_solutions = np.zeros((n_segments, 8))
    #     for idx in range(n_segments):
    #         t = (idx + 1) / n_segments  # Interpolation parameter in range of [0, 1]
    #         pose_interpolated = np.zeros(7)
    #         pose_interpolated[:3] = self._ops_coords[:3] + t*translation  # Segment end position
    #         pose_interpolated[3:] = utils.slerp(self._ops_coords[3:], target[3:], t)  # Segment end quaternion
    #         new_jnt_vec, ik_sol = self.i_kin(pose_interpolated, prev_jnt_vec=previous_jnt_vec, wrist_flip=self.wrist_flip)
    #         jnt_solutions[idx] = new_jnt_vec.copy()
    #         previous_jnt_vec = new_jnt_vec.copy()
    #
    #         if ik_sol != IK_SOLUTION['SUCCESS']:
    #             print(f"move_ops_lin: IK fail: {ik_sol}")
    #             return False, g_code_list
    #
    #
    #     # All interpolation points computed successfully
    #     # Write to file if file is open
    #     if self.gc_writer.file_is_open():
    #         self.gc_writer.write_comment("Linear move in operational space")
    #
    #     # Generate G-code
    #     for jnt_pos in jnt_solutions:
    #         segment_success, g_code = self._move_jnt(jnt_pos, segment_time, incremental=False)
    #         success = success and segment_success
    #         g_code_list.append(g_code)
    #
    #     return success, g_code_list

    def write_g_code(self, mot_vec: np.ndarray, mot_vec_current: np.ndarray, time: float, incremental: bool=False) -> bool:
        """ Generates a G-code with units of deg, mm and minutes and emits it to g_code_generated signal
        @param mot_vec: 8-vector of motor absolute coordinates in controller distance units (deg for angular, mm for linear)
        @param mot_vec_current: 8-vector of current motor positions. Needed for computing motion distance.
        @param time: Motion duration in seconds
        @param incremental: Bool, True for incremental motor coordinates, False for absolute.
        @return: True if generation of motion successful
        """
        # Movement difference vector
        mot_diff_vec = mot_vec - mot_vec_current

        # Compute movement speed. Unit is a GRBL speed unit. deg/min for revolute, mm/min for linear.
        if time <= 0:
            print("Manager: Non positive movement time")
            return False

        distance = LA.norm(mot_diff_vec)
        feedrate = distance / time * 60  # Desired feed rate as deg/min and mm/min
        # print(f"Segment length mot.space: {distance:.3f}\tTime: {time:.3f}\tFeedrate: {feedrate}")

        # Scale down speeds such that no motor exceeds its maximum speed
        mot_unit_vec = np.abs(mot_diff_vec / distance)
        for idx in range(8):
            if mot_unit_vec[idx] > 1e6:  # Avoid near zero denominator
                feedrate = min(MOTOR_MAX_SPEED[f"M{idx+1}"] / mot_unit_vec[idx], feedrate)

        # Write G-code for motor motion
        g_code = self.gc_writer.move_linear(
            x=float(mot_vec[0]),
            y=float(mot_vec[1]),
            z=float(mot_vec[2]),
            a=float(mot_vec[3]),
            b=float(mot_vec[4]),
            c=float(mot_vec[5]),
            u=float(mot_vec[6]),
            v=float(mot_vec[7]),
            feedrate=feedrate)
        print(f"write_g_code: {g_code}")

        self.g_code_generated.emit(g_code)

        return True

    # Getters
    @property
    def mot_coords(self):
        return self._manipulator.mot_coords

    @property
    def jnt_coords(self):
        return self._manipulator.jnt_coords

    @property
    def ops_coords(self):
        return self._manipulator.ops_coords

    @property
    def singular_vals_vecs_trans(self):
        return self._manipulator.singular_vals_vecs_trans

    @property
    def singular_vals_vecs_rot(self):
        return self._manipulator.singular_vals_vecs_rot

    def update_status(self, mot_list: list):
        """ Update system status and return by dictionary.
        @param status: str, controller status
        @param mot_list: List of 8 floats, motor positions as degrees
        """
        mot_vec = np.deg2rad(np.array(mot_list))
        self._manipulator.update_state(mot_vec[:6])  # 6 axis
        self._linear_base.update_state(mot_vec[6:7])  # 1 axis

        mot_vec_deg = np.rad2deg(self._manipulator.mot_coords)
        ops_vec_base = self._manipulator.ops_coords
        rot_mat_tool_wrt_base = utils.quat2rot_mat(ops_vec_base[3:])

        zyz_tool_wrt_base = utils.rot2zyz(rot_mat_tool_wrt_base, phi_prev=self._manipulator.jnt_coords[3], psi_prev=self._manipulator.jnt_coords[5])[0]

        sing_vals_trans, sing_vecs_trans = self._manipulator.singular_vals_vecs_trans
        sing_vals_rot, sing_vecs_rot = self._manipulator.singular_vals_vecs_rot

        ops_vec_world = ops_vec_base.copy()
        ops_vec_world[:3] = (self._T_base_wrt_world @ np.append(ops_vec_base[:3], 1.))[:3]  # Position w.rot.t. world
        quat_tool_wrt_base = ops_vec_base[3:]
        quat_tool_wrt_world = utils.quat_multiply(self._quat_base_wrt_world, quat_tool_wrt_base)
        ops_vec_world[3:] = quat_tool_wrt_world
        # rot_mat_world = utils.quat2rot_mat(quat_tool_wrt_world)
        rot_mat_world = utils.quat2rot_mat(self._quat_base_wrt_world) @ rot_mat_tool_wrt_base
        zyz_euler_world = utils.rot2zyz(rot_mat_world, phi_prev=self._manipulator.jnt_coords[3],
                                        psi_prev=self._manipulator.jnt_coords[5])[0]

        # Condition
        cond_world = self._manipulator._compute_condition(rot_mat_world)
        cond_base = self._manipulator._compute_condition(np.eye(3))
        cond_tool = self._manipulator._compute_condition(rot_mat_tool_wrt_base)

        # TODO: Fix this bandate
        system_mot_vec_deg = mot_list.copy()
        system_jnt_vec_deg = np.zeros(8)
        system_jnt_vec_deg[:6] = np.rad2deg(self._manipulator.jnt_coords)
        state = {
            'mot_coords_deg': system_mot_vec_deg,
            'jnt_coords_deg': system_jnt_vec_deg,
            'ops_coords_base': ops_vec_base,
            'ops_coords_world': ops_vec_world,
            'zyz_euler_base': zyz_tool_wrt_base,
            'zyz_euler_world': zyz_euler_world,
            'condition_world': cond_world,
            'condition_base': cond_base,
            'condition_tool': cond_tool,
            'sing_vals_trans': sing_vals_trans,
            'sing_vecs_trans': sing_vecs_trans,
            'sing_vals_rot': sing_vals_rot,
            'sing_vecs_rot': sing_vecs_rot,
        }
        return state

    def move_single_jnt_linear_axis(self, jnt_idx: int, distance: float, speed: float, incremental=False) -> bool:
        if (jnt_idx + 1) > N_LIN_JNT:
            print("Invalid joint index")
            return False

        # Fetch temporary absolute joint coordinates
        jnt_vec_current = self._linear_base.temp_jnt_coords

        # Construct target joint vector
        jnt_vec_target = jnt_vec_current.copy()
        jnt_vec_target[jnt_idx] = distance

        # Compute movement time
        move_time_s = abs((distance - jnt_vec_current[jnt_idx]) / speed)

        mot_vec = self._linear_base.move_jnt(jnt_vec_target)
        if mot_vec is None:
            return False

        # Send to g_code_writer
        jnt_vec_target_mm = np.zeros(8)
        jnt_vec_target_mm[6:7] = 1000 * mot_vec
        jnt_vec_current_mm = np.zeros(8)
        jnt_vec_current_mm[6:7] = 1000 * jnt_vec_current
        self.write_g_code(jnt_vec_target_mm, jnt_vec_current_mm, move_time_s)

        return True

    def move_single_jnt_manipulator(self, jnt_idx: int, angle: float, speed: float, incremental=False) -> bool:
        """ Gets a single joint motion instruction from API. Let manipulator and linear axis verify motion,
        then propagate forward to g-code writer.
        @param jnt_idx: Index of joint to move (0-7)
        @param angle: Angle to move to, radians
        @param speed: , Speed to move at, radians/second
        @param incremental: Only absolute motion for now
        """
        if (jnt_idx+1) > N_REV_JNT:
            print("Invalid joint index")
            return False

        # Fetch temporary absolute joint coordinates
        jnt_vec_current = self._manipulator.temp_jnt_coords

        # Construct target joint vector
        jnt_vec_target = jnt_vec_current.copy()
        jnt_vec_target[jnt_idx] = angle

        # Compute movement time
        move_time_s = abs((angle - jnt_vec_current[jnt_idx]) / speed)
        print(f"New: {np.rad2deg(angle)}\tCurrent: {np.rad2deg(jnt_vec_current[jnt_idx])}")

        mot_vec = self._manipulator.move_jnt(jnt_vec_target)
        if mot_vec is None:
            return False

        # Send to g_code_writer
        jnt_vec_target_deg = np.zeros(8)
        jnt_vec_target_deg[:6] = np.rad2deg(mot_vec)
        jnt_vec_current_deg = np.zeros(8)
        jnt_vec_current_deg[:6] = np.rad2deg(jnt_vec_current)
        self.write_g_code(jnt_vec_target_deg, jnt_vec_current_deg, move_time_s)

        return True

    def move_ops(self, pose: np.ndarray, time: float, incremental: bool = False,
                 shoulder_flip: bool = False, elbow_down: bool = False, wrist_flip: bool = False) -> bool:
        success, g_code = self._manipulator.move_ops(pose, time, incremental, shoulder_flip, elbow_down, wrist_flip)
        if not success:
            return False
        self.g_code_generated.emit(g_code)
        return True

    def move_ops_lin_7d(self, ops_vec: np.ndarray, time: float, criteria: int, incremental: bool = False) -> bool:
        """
        Numerical inverse kinematics
        """
        # q_dot = J^-1 * x_dot

    def translate_tool(self, direction_vec: tuple[int, int, int], distance: float, speed: float, frame: str, millimeters=False) -> bool:
        """ Creates a pure translation along any axis in any frame.
        :param direction_vec: Translation axis, any length
        :param distance: Translation distance, meters by default
        :param speed: Translation speed, meters/second
        :param frame: Frame direction vector is described in. "World", "Base" or "Tool"
        :param millimeters: Units for distance
        """
        # Compute list of G-code for translational move
        success, g_code_list = self._manipulator.translate_tool(direction_vec, distance, speed, frame, millimeters)
        if not success:
            return False

        # Send G-code to serial
        for line in g_code_list:
            self.g_code_generated.emit(line)

        return True

    def rotate_tool(self, direction_vec: tuple[int, int, int], angle: float, speed: float, frame: str, degrees=False) -> bool:
        """ Creates a pure rotation around any axis in any frame.
        :param direction_vec: Rotation axis, any length
        :param angle: Rotation angle, radians by default
        :param speed: Rotation speed, radians/second
        :param frame: Frame respect to which direction vector is described. "World", "Base" or "Tool"
        :param degrees: Unit of angle
        """
        # Compute list of G-code for rotational move
        success, g_code_list = self._manipulator.rotate_tool(direction_vec, angle, speed, frame, degrees)
        if not success:
            return False

        # Send G-code to serial
        for line in g_code_list:
            self.g_code_generated.emit(line)

        return True

    def reset(self):
        self._manipulator.reset()

    def execute(self):
        if self.my_program is not None:
            instructions = self.my_program.instructions  # list
            move_joint: Instruction = instructions[0]
            target_pose = move_joint.target.pose
            self.move_ops(target_pose, 10)
