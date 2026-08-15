""" Wrapper class for Manipulator

Author: Rainer Meyer, rot.meyer494@gmail.com
"""
import config
from backend.manipulator import Manipulator
from backend.linear_axis import LinearAxis
from backend.g_code_writer import GCodeWriter
from backend.system_state import SystemState, ControllerState
from robot_math.pose import Pose
from config import *
import utils

from typing import Tuple
import numpy as np
import numpy.linalg as LA
from PyQt6.QtCore import QObject, pyqtSignal

from robot_math.quaternion import Quaternion


class RobotSystem(QObject):

    g_code_generated = pyqtSignal(str)
    send_terminal = pyqtSignal(str)
    sgn_speed_throttled = pyqtSignal(float)

    def __init__(self):

        super().__init__()

        self._manipulator = Manipulator()
        self._linear_axis = LinearAxis()

        self.gc_writer = GCodeWriter()

        self.previous_pose: Pose = Pose.identity()
        self.speed_linear_prev = 0.
        self.speed_angular_prev = 0.

        # self.executor = MotionExecutor()

        self.sys_state: SystemState = SystemState(self._manipulator.state, self._linear_axis.state)

    # def move_lin_7d(self, end_pose: Pose, stepping_rate: float = 1e-4) -> bool:
    #     """ Numerical inverse kinematics to solve 7 joints for given pose and an additional criteria
    #     @param pose: Desired end posture
    #     @param criteria: Additional criteria for specific joint solution
    #     """
    #
    #     # pos_error: float  = current_pose.distance(end_pose)
    #     # ang_error: float = current_pose.quaternion.angle(end_pose.quaternion)
    #
    #     converge = False
    #     iters = 0
    #     while not converge and iters < 1000:
    #         current_pose: Pose = self.sys_state.queued_pose
    #         if current_pose.distance(end_pose) < 0.001:  # 1mm
    #             return True
    #         trans_dir: np.ndarray = current_pose.translation_direction(end_pose)
    #         rot_dir: np.ndarray = current_pose.rotation_direction(end_pose)
    #         dx: np.ndarray = stepping_rate * np.hstack((trans_dir, rot_dir)).T  # 6-vector
    #
    #         # Fetch jacobian and do pseudo inverse
    #         J = self.sys_state.queued_jacobian
    #         J_pinv = LA.pinv(J)
    #         dq = J_pinv @ dx
    #         jnt_vec = self.sys_state.queued_motors
    #         jnt_vec[:7] += dq
    #         self.sys_motor_move(jnt_vec)
    #         iters += 1
    #
    #     return True

    def move_null(self, null_vec: np.ndarray, criteria) -> bool:
        ...
        # quat_err = utils.quat_multiply(quat_desired, quat_current.inv)
        # Quaternion error to rotation vector (for small errors only)
        # rot_vec = 2 * quat_err[1:]  # Small angle approximation
        # delta_x = np.hstack((pos_err, rot_vec)).T

        # current_pose: Pose = self.sys_state.queued_pose
        # if current_pose.distance(end_pose) > 0.010:
        #     return False
        #
        # max_iters = 200
        # iteration = 0
        # temp_jnt_vec = np.zeros(8)
        # while iteration < max_iters:
        #     # Unit directions of motion.
        #     trans_dir: np.ndarray = current_pose.translation_direction(end_pose)
        #     rot_dir: np.ndarray = current_pose.rotation_direction(end_pose)
        #
        #     # Fetch jacobian and do pseudo inverse
        #     J = self.sys_state.queued_jacobian
        #     if LA.det(J) < 0.001:
        #         return False
        #     J_pinv = LA.pinv(J)
        #     delta_q = J_pinv @ delta_x
        #     iters += 1
        #
        # return True

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
        # Current queued motor position vector. Degrees and millimeters.
        sys_mot_vec_queued_ctrl_units: np.ndarray = self.sys_state.queued.motor_state_ctrl_units

        # Construct absolute target vector
        sys_mot_vec_target_ctrl_units = sys_mot_vec_queued_ctrl_units.copy()
        if incremental:  # Add
            if mot_vec_manipulator is not None:
                sys_mot_vec_target_ctrl_units[:6] += np.rad2deg(mot_vec_manipulator[:6].copy())
            if mot_vec_linear_axis is not None:
                sys_mot_vec_target_ctrl_units[6:7] += 1000 * mot_vec_linear_axis.copy()

        else:  # Override
            if mot_vec_manipulator is not None:
                sys_mot_vec_target_ctrl_units[:6] = np.rad2deg(mot_vec_manipulator.copy())  # Target position in degrees
            if mot_vec_linear_axis is not None:
                sys_mot_vec_target_ctrl_units[6:7] = 1000 * mot_vec_linear_axis.copy()  # Target position in millimeters

        # Normalized direction vector
        delta_mot_vec = sys_mot_vec_target_ctrl_units - sys_mot_vec_queued_ctrl_units
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
        feedrate_throttle_factor: float = 1.
        mot_dir_vec = np.abs(dir_vec)
        for idx in range(len(config.MOTOR_MAX_SPEED)):
            if mot_dir_vec[idx] > 1e6:  # Avoid near zero denominator
                feedrate_throttled = min(config.MOTOR_MAX_SPEED[f"M{idx+1}"] / mot_dir_vec[idx], feedrate)
                feedrate_throttle_factor = min(feedrate_throttle_factor, feedrate_throttled / feedrate)
                feedrate = feedrate_throttled
        self.sgn_speed_throttled.emit(feedrate_throttle_factor)

        # Write G-code for motor motion and send to serial queue.
        g_code = self.gc_writer.move(x=float(sys_mot_vec_target_ctrl_units[0]), y=float(sys_mot_vec_target_ctrl_units[1]),
                                            z=float(sys_mot_vec_target_ctrl_units[2]), a=float(sys_mot_vec_target_ctrl_units[3]),
                                            b=float(sys_mot_vec_target_ctrl_units[4]), c=float(sys_mot_vec_target_ctrl_units[5]),
                                            u=float(sys_mot_vec_target_ctrl_units[6]), v=float(sys_mot_vec_target_ctrl_units[7]),
                                            feedrate=feedrate, rapid=False)

        self.sys_state.queued.motor_state_ctrl_units = sys_mot_vec_target_ctrl_units
        self.g_code_generated.emit(g_code)
        # print(f"write_g_code: {g_code}")

        return True

    def move_jnt(self, jnt_vec: np.ndarray, time: float | None = None, speed: float | None = None, degrees: bool = False) -> bool:
        """ Move 8-joint
        @:param jnt_vec: 8-vector, radians and meters
        @:param time: Motion time in seconds
        @:param speed: Motion speed in rad/s
        """
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

    def move_ops(self, target_pose: Pose, time_seconds: float) -> bool:
        """ Motor space interpolated motion to given posture.
        :param target_pose:
        :param time_seconds:
        :return:
        """
        mot_vec = self._manipulator.move_ops(target_pose=target_pose)
        if mot_vec is None:
            return False

        self.sys_motor_move(mot_vec_manipulator=mot_vec, time=time_seconds)
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
        ret = self._manipulator.translate_tool(direction_vec, distance, speed, frame)
        if ret is None:
            self.send_terminal.emit("Translation failed.")
            return False

        # Unpack motor values and segment time.
        mot_vecs, segment_time = ret

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
        ret = self._manipulator.rotate_tool(direction_vec, angle, speed, frame)
        if ret is None:
            self.send_terminal.emit("Rotation failed.")
            return False

        # Unpack motor values and segment time.
        mot_vecs, segment_time = ret

        # Send G-code to serial
        for vec in mot_vecs:
            self.sys_motor_move(mot_vec_manipulator=vec, time=segment_time)

        return True

    def reset(self):
        self._manipulator.reset()

    def toggle_feed_hold(self):
        if self.sys_state.controller == ControllerState.HOLD:
            self.g_code_generated.emit(self.gc_writer.cycle_start())
        elif self.sys_state.controller == ControllerState.IDLE:
            self.g_code_generated.emit(self.gc_writer.cycle_start())
        elif self.sys_state.controller == ControllerState.CYCLE:
            self.g_code_generated.emit(self.gc_writer.feed_hold())

    def update_status(self, status: str, mot_list: list, delta_t: float):
        """ Update system status and return by dictionary.
        @param status: str, controller status.
        @param mot_list: List of 8 floats, motor positions as degrees.
        @param delta_t: Time in seconds since last update.
        """
        match status:
            case "Idle":
                self.sys_state.controller = ControllerState.IDLE
            case "Run":
                self.sys_state.controller = ControllerState.CYCLE
            case "Hold":
                self.sys_state.controller =  ControllerState.HOLD
            case "Home":
                self.sys_state.controller = ControllerState.HOMING
            case "Alarm":
                self.sys_state.controller = ControllerState.ALARM
            case "Check":
                self.sys_state.controller = ControllerState.CHECK
            case "Door":
                self.sys_state.controller = ControllerState.SAFETY_DOOR

        # Real motor values reported by controller
        mot_vec = np.array(mot_list)  # Degrees and millimeters

        # Update states
        self._manipulator.state.mcu.motor_state =np.deg2rad(mot_vec[:N_REV_JNT])
        self._linear_axis.state.mcu.joint_state =0.001 * mot_vec[N_REV_JNT:N_JNT]

        tool_wrt_base: Pose = self._manipulator.state.mcu.ops_state
        base_wrt_world = self._linear_axis.state.mcu.pose
        tool_wrt_world = base_wrt_world.compose(tool_wrt_base)

        sing_vals_trans, sing_vecs_trans = self._manipulator.state.mcu.singular_data_translation
        sing_vals_rot, sing_vecs_rot = self._manipulator.state.mcu.singular_data_rotation

        # Condition
        # TODO: Fix base and world
        cond_world = self._manipulator._compute_condition(np.eye(3))
        cond_base = self._manipulator._compute_condition(np.eye(3))
        cond_tool = self._manipulator._compute_condition(tool_wrt_base.rot_mat)

        jnt_vec_deg = np.zeros(8)
        jnt_vec_deg[:6] = np.rad2deg(self._manipulator.state.mcu.joint_state)
        jnt_vec_deg[6:7] = 1000 * self._linear_axis.state.mcu.joint_state

        # delta_t = 0.1
        alpha = 0.95
        delta_x = LA.norm(tool_wrt_base.position - self.previous_pose.position)
        speed_linear: float = alpha*utils.m_s2mm_min(delta_x / delta_t) + (1-alpha)*self.speed_linear_prev
        speed_angular: float = alpha*utils.rad_sec2deg_min(abs(tool_wrt_base.quaternion.angle(self.previous_pose.quaternion)) / delta_t) + (1-alpha)*self.speed_angular_prev
        self.speed_linear_prev, self.speed_angular_prev = speed_linear, speed_angular

        self.previous_pose = tool_wrt_base

        state = {
            'status': status,
            'mot_coords_deg': np.array(mot_list),
            'jnt_coords_deg': jnt_vec_deg,
            'ops_coords_base': tool_wrt_base,
            'ops_coords_world': tool_wrt_world,
            'link_poses': self._manipulator.state.mcu.link_poses,
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