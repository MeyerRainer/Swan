"""
Wrapper class for Manipulator
Author: Rainer Meyer, rot.meyer494@gmail.com
"""

from backend.manipulator import Manipulator
from backend.linear_axis import LinearAxis
from backend.g_code_writer import GCodeWriter
from robot_math.pose import Pose
from config import *
import utils
import numpy as np
import numpy.linalg as LA
from PyQt6.QtCore import QObject, pyqtSignal


class RobotSystem(QObject):

    g_code_generated = pyqtSignal(str)
    send_terminal = pyqtSignal(str)

    def __init__(self):

        super().__init__()

        self._manipulator = Manipulator()
        self._linear_axis = LinearAxis()

        self.gc_writer = GCodeWriter()

        self.previous_pose: Pose = Pose.identity()
        self.speed_linear_prev = 0.
        self.speed_angular_prev = 0.

        # self.sys_state: SystemState = SystemState(self._manipulator.state, self._linear_axis.state)

    # def i_kin_7d(self, end_pose: Pose, quat_err: np.ndarray, criteria) -> Tuple[np.ndarray | None, IK_SOLUTION]:
    #     """ Numerical inverse kinematics to solve 7 joints for given pose and an additional criteria
    #     @param pose: Desired end posture
    #     @param criteria: Additional criteria for specific joint solution
    #     """
    #     current_pose: Pose = self._sys_pose.copy()
    #     # TODO: Quat error in caller function
    #     # quat_err = utils.quat_multiply(quat_desired, quat_current.inv)
    #     # Quaternion error to rotation vector (for small errors only)
    #     rot_vec = 2 * quat_err[1:]  # Small angle approximation
    #     delta_x = np.hstack((pos_err, rot_vec)).T
    #
    #     converge = False
    #     iters = 0
    #     while not converge and iters > 200:
    #         # Error
    #         pos_err: np.ndarray = end_pose.position - current_pose.position
    #         rot_err: np.ndarray = end
    #         # Fetch jacobian and do pseudo inverse
    #         J = self._sys_jacobian.copy()
    #         J_pinv = LA.pinv(J)
    #         delta_q = J_pinv @ delta_x

    def get_queue_motor(self):
        """ Get motor coordinates currently queued for motion. Degrees and millimeters. """
        sys_queue_motor = np.zeros(8)
        sys_queue_motor[:6] = np.rad2deg(self._manipulator.state.queued.motor_state)
        sys_queue_motor[6:7] = 1000. * self._linear_axis.state.queued.joint_state
        return sys_queue_motor

    def write_g_code(self, mot_vec: np.ndarray, feedrate: float) -> bool:
        """ Generates a G-code with units of deg, mm and minutes and emits it to g_code_generated signal
        @:param mot_vec: 8-vector of motor absolute coordinates in controller distance units (deg for angular, mm for linear)
        @:param feedrate: Feedrate. deg/min and mm/min
        @:return: True if generation of motion successful
        """

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

        self.g_code_generated.emit(g_code)
        # print(f"write_g_code: {g_code}")

        return True

    def sys_motor_move(self, mot_vec_manipulator: np.ndarray | None = None, mot_vec_linear_axis: np.ndarray | None = None,
                       time: float | None = None, speed: float | None = None, incremental=False) -> bool:
        """ Constructs an 8-vector of absolute motor coordinates to move to and a unit vector pointing
        towards the motion direction in motor space.
        @:param mot_vec_manipulator: Desired absolute coordinates for manipulator in radians
        @:param mot_vec_linear_base: Desired absolute coordinates for linear base in meters
        @:param time: Motion time in seconds
        @:param speed: Motion speed in rad/s
        @:return: True if move sent to queue
        """
        sys_mot_vec_current: np.ndarray = self.get_queue_motor()  # Current queued motor position vector. Degrees and millimeters.

        # Construct absolute target vector
        sys_mot_vec_target = sys_mot_vec_current.copy()
        if incremental:  # Add
            if mot_vec_manipulator is not None:
                sys_mot_vec_target[:6] += np.rad2deg(mot_vec_manipulator[:6].copy())
            if mot_vec_linear_axis is not None:
                sys_mot_vec_target[6:7] += 1000 * mot_vec_linear_axis.copy()

        else:  # Override
            if mot_vec_manipulator is not None:
                sys_mot_vec_target[:6] = np.rad2deg(mot_vec_manipulator.copy())  # Target position in degrees
            if mot_vec_linear_axis is not None:
                sys_mot_vec_target[6:7] = 1000 * mot_vec_linear_axis.copy()  # Target position in millimeters

        # Normalized direction vector
        delta_mot_vec = sys_mot_vec_target - sys_mot_vec_current
        delta_mot_vec_norm = LA.norm(delta_mot_vec)
        dir_vec = delta_mot_vec / delta_mot_vec_norm

        # Define feedrate
        if speed is not None:
            feedrate: float = utils.rad_sec2deg_min(speed)
        elif time is not None:
            feedrate: float = 60 * delta_mot_vec_norm / time  # mm/min and deg/min
        else:
            self.send_terminal.emit("No speed or time given.")
            return False

        # Scale down speeds such that no motor exceeds its maximum speed
        mot_dir_vec = np.abs(dir_vec)
        for idx in range(8):
            if mot_dir_vec[idx] > 1e6:  # Avoid near zero denominator
                feedrate = min(MOTOR_MAX_SPEED[f"M{idx+1}"] / mot_dir_vec[idx], feedrate)

        # Set queued state
        self._manipulator.state.queued.motor_state = np.deg2rad(sys_mot_vec_target[:6])

        return self.write_g_code(sys_mot_vec_target, feedrate)

    def move_jnt(self, jnt_vec: np.ndarray, time: float | None = None, speed: float | None = None, degrees: bool = False) -> bool:
        """ Move 8-joint
        @:param jnt_vec: 8-vector, radians and meters
        @:param time: Motion time in seconds
        @:param speed: Motion speed in rad/s
        """
        print(f"move_jnt called with jnt_vec: {jnt_vec}")
        mot_vec_manipulator_rad = jnt_vec[:6]
        mot_vec_linear_axis_m = jnt_vec[6:7]

        mot_vec_man = self._manipulator.move_jnt(mot_vec_manipulator_rad, degrees)
        if mot_vec_man is None:
            return False
        mot_vec_lin = self._linear_axis.move_jnt(mot_vec_linear_axis_m)
        if mot_vec_lin is None:
            return False

        # Send to serial
        self.sys_motor_move(mot_vec_manipulator=mot_vec_man, mot_vec_linear_axis=mot_vec_lin, time=time, speed=speed)

        return True

    def move_ops_lin_7d(self, ops_vec: np.ndarray, time: float, criteria: int, incremental: bool = False) -> bool:
        """
        Numerical inverse kinematics
        """
        # q_dot = J^-1 * x_dot


    def translate_tool(self, direction_vec: tuple[int, int, int], distance: float, speed: float, frame: str) -> bool:
        """ Creates a pure translation along any axis in any frame.
        :param direction_vec: Translation axis, any length
        :param distance: Translation distance, meters
        :param speed: Translation speed, meters/second
        :param frame: Frame direction vector is described in. "World", "Base" or "Tool"
        """
        # Compute list of G-code for translational move
        mot_vecs, segment_time = self._manipulator.translate_tool(direction_vec, distance, speed, frame)
        if mot_vecs is None:
            self.send_terminal("Translation failed.")
            return False

        # Send G-code to serial
        for vec in mot_vecs:
            # self.write_g_code(vec, mot_vec_prev, segment_time)
            self.sys_motor_move(mot_vec_manipulator=vec, time=segment_time)

        return True

    def rotate_tool(self, direction_vec: tuple[int, int, int], angle: float, speed: float, frame: str) -> bool:
        """ Creates a pure rotation around any axis in any frame.
        :param direction_vec: Rotation axis, any length
        :param angle: Rotation angle, radians
        :param speed: Rotation speed, radians/second
        :param frame: Frame respect to which direction vector is described. "World", "Base" or "Tool"
        """
        # Compute list of G-code for rotational move
        mot_vecs, segment_time = self._manipulator.rotate_tool(direction_vec, angle, speed, frame)
        if mot_vecs is None:
            self.send_terminal("Rotation failed.")
            return False

        # Send G-code to serial
        for vec in mot_vecs:
            self.sys_motor_move(mot_vec_manipulator=vec, time=segment_time)

        return True

    def reset(self):
        self._manipulator.reset()

    def update_status(self, mot_list: list, delta_t: float):
        """ Update system status and return by dictionary.
        @param status: str, controller status.
        @param mot_list: List of 8 floats, motor positions as degrees.
        @param delta_t: Time in seconds since last update.
        """
        # Real motor values reported by controller
        mot_vec = np.array(mot_list)  # Degrees and millimeters

        # Update states
        self._manipulator.update_state(np.deg2rad(mot_vec[:6]))  # 6 axis
        self._linear_axis.update_state(0.001 * mot_vec[6:7])  # 1 axis

        ops_pose_base: Pose = self._manipulator.state.mcu.ops_state


        # TODO: Bring back Pose and understand this
        # pose_base_to_world = self._linear_axis.state.mcu.base_pose.inverse()
        # ops_pose_world: Pose = ops_pose_base.relative_to(pose_base_to_world)
        ops_pose_world = ops_pose_base.copy()
        ops_pose_world.position += self._linear_axis.state.mcu.position

        sing_vals_trans, sing_vecs_trans = self._manipulator.state.mcu.singular_data_translation
        sing_vals_rot, sing_vecs_rot = self._manipulator.state.mcu.singular_data_rotation

        # Condition
        # TODO: Fix base and world
        cond_world = self._manipulator._compute_condition(np.eye(3))
        cond_base = self._manipulator._compute_condition(np.eye(3))
        cond_tool = self._manipulator._compute_condition(ops_pose_base.rot_mat)

        jnt_vec_deg = np.zeros(8)
        jnt_vec_deg[:6] = np.rad2deg(self._manipulator.state.mcu.joint_state)
        jnt_vec_deg[6:7] = 1000 * self._linear_axis.state.mcu.joint_state

        # delta_t = 0.1
        alpha = 0.9
        delta_x = LA.norm(ops_pose_base.position - self.previous_pose.position)
        speed_linear: float = alpha*utils.m_s2mm_min(delta_x / delta_t) + (1-alpha)*self.speed_linear_prev
        speed_angular: float = alpha*utils.rad_sec2deg_min(abs(ops_pose_base.quaternion.angle(self.previous_pose.quaternion)) / delta_t) + (1-alpha)*self.speed_angular_prev
        self.speed_linear_prev, self.speed_angular_prev = speed_linear, speed_angular

        self.previous_pose = ops_pose_base

        state = {
            'mot_coords_deg': np.array(mot_list),
            'jnt_coords_deg': jnt_vec_deg,
            'ops_coords_base': ops_pose_base,
            'ops_coords_world': ops_pose_world,
            'condition_world': cond_world,
            'condition_base': cond_base,
            'condition_tool': cond_tool,
            'sing_vals_trans': sing_vals_trans,
            'sing_vecs_trans': sing_vecs_trans,
            'sing_vals_rot': sing_vals_rot,
            'sing_vecs_rot': sing_vecs_rot,
            'speed_linear': speed_linear,
            'speed_angular': speed_angular,
        }
        return state