""" Wrapper class for Manipulator

Author: Rainer Meyer, rot.meyer494@gmail.com
"""
from PyQt6.QtCore import QObject, pyqtSignal
import numpy as np
from typing import Optional

from robot_manipulator.system_state import SystemState, ControllerState
from robot_manipulator.manipulator import Manipulator
from robot_manipulator.linear_axis import LinearAxis
from robot_manipulator.gc_writer import GCodeWriter
from robot_math.pose import Pose
from robot_math import utils
import config


class RobotSystem(QObject):

    g_code_generated = pyqtSignal(str)
    send_terminal = pyqtSignal(str)
    sgn_speed_throttled = pyqtSignal(float)

    def __init__(self):

        super().__init__()

        self._manipulator = Manipulator()
        self._linear_axis = LinearAxis()
        self._gc_writer = GCodeWriter()
        self.state: SystemState = SystemState(self._manipulator.state, self._linear_axis.state)

        # TODO: Delete.
        self.previous_pose: Pose = Pose.identity()
        self.speed_linear_prev = 0.
        self.speed_angular_prev = 0.

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
    #         current_pose: Pose = self.state.queued_pose
    #         if current_pose.distance(end_pose) < 0.001:  # 1mm
    #             return True
    #         trans_dir: np.ndarray = current_pose.translation_direction(end_pose)
    #         rot_dir: np.ndarray = current_pose.rotation_direction(end_pose)
    #         dx: np.ndarray = stepping_rate * np.hstack((trans_dir, rot_dir)).T  # 6-vector
    #
    #         # Fetch jacobian and do pseudo inverse
    #         J = self.state.queued_jacobian
    #         J_pinv = LA.pinv(J)
    #         dq = J_pinv @ dx
    #         jnt_vec = self.state.queued_motors
    #         jnt_vec[:7] += dq
    #         self.sys_motor_move(jnt_vec)
    #         iters += 1
    #     return True
    #
    # def move_null(self, null_vec: np.ndarray, criteria) -> bool:
    #     quat_err = utils.quat_multiply(quat_desired, quat_current.inv)
    #     Quaternion error to rotation vector (for small errors only)
    #     rot_vec = 2 * quat_err[1:]  # Small angle approximation
    #     delta_x = np.hstack((pos_err, rot_vec)).T
    #
    #     current_pose: Pose = self.state.queued_pose
    #     if current_pose.distance(end_pose) > 0.010:
    #         return False
    #
    #     max_iters = 200
    #     iteration = 0
    #     temp_jnt_vec = np.zeros(8)
    #     while iteration < max_iters:
    #         # Unit directions of motion.
    #         trans_dir: np.ndarray = current_pose.translation_direction(end_pose)
    #         rot_dir: np.ndarray = current_pose.rotation_direction(end_pose)
    #
    #         # Fetch jacobian and do pseudo inverse
    #         J = self.state.queued_jacobian
    #         if LA.det(J) < 0.001:
    #             return False
    #         J_pinv = LA.pinv(J)
    #         delta_q = J_pinv @ delta_x
    #         iters += 1
    #     return True

    def plan(self) -> None:
        pose_world: Pose = self._manipulator.state.planner.link_nodes["ToolFrame"].pose  # World frame
        pose_base: Pose = pose_world.relative_to(self.state.mcu.linear_axis.pose)

        mot_vec = self._manipulator.request_cartesian_move(pose_base)
        if mot_vec is None:
            return
        self._manipulator.state.planner.motor_state = mot_vec
        self._linear_axis.state.planned.joint_state = np.array([0.])

    def plan_8d(self) -> None:
        pose_world: Pose = self._manipulator.state.planner.link_nodes["ToolFrame"].pose  # World frame

        mot_vec: np.ndarray = self.move_cartesian_8d(pose_world)
        if mot_vec is None:
            return

        self.state.planned.motor_state = mot_vec

    def sys_queue_move(self, mot_vec_manipulator: Optional[np.ndarray] = None,
                       mot_vec_linear_axis: Optional[np.ndarray] = None,
                       time: Optional[float] = None,
                       speed: Optional[float] = None,
                       incremental=False) -> bool:
        """ Constructs an 8-vector of absolute motor coordinates in controller generalized units,
        limits the feedrate and sends the vector to motion queue.
        @:param mot_vec_manipulator: Desired absolute coordinates for manipulator in radians
        @:param mot_vec_linear_base: Desired absolute coordinates for linear base in meters
        @:param time: Motion time in seconds
        @:param speed: Motion speed in rad/s
        @:return: True if move sent to queue
        """
        # Current queued motor position vector. Degrees and millimeters.
        sys_mot_vec_queued_ctrl_units: np.ndarray = self.state.queued.motor_state_ctrl_units

        # Construct absolute target vector
        sys_mot_vec_target_ctrl_units = sys_mot_vec_queued_ctrl_units.copy()
        if incremental:  # Add
            if mot_vec_manipulator is not None:
                sys_mot_vec_target_ctrl_units[:config.N_REV_JNT] += np.rad2deg(mot_vec_manipulator[:config.N_REV_JNT].copy())
            if mot_vec_linear_axis is not None:
                sys_mot_vec_target_ctrl_units[config.N_REV_JNT:config.N_JNT] += 1000 * mot_vec_linear_axis.copy()

        else:  # Override
            if mot_vec_manipulator is not None:
                sys_mot_vec_target_ctrl_units[:config.N_REV_JNT] = np.rad2deg(mot_vec_manipulator.copy())  # Target position in degrees
            if mot_vec_linear_axis is not None:
                sys_mot_vec_target_ctrl_units[config.N_REV_JNT:config.N_JNT] = 1000 * mot_vec_linear_axis.copy()  # Target position in millimeters

        # Normalized direction vector
        delta_mot_vec = sys_mot_vec_target_ctrl_units - sys_mot_vec_queued_ctrl_units
        delta_mot_vec_norm = np.linalg.norm(delta_mot_vec)
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
        g_code = self._gc_writer.move(x=float(sys_mot_vec_target_ctrl_units[0]), y=float(sys_mot_vec_target_ctrl_units[1]),
                                      z=float(sys_mot_vec_target_ctrl_units[2]), a=float(sys_mot_vec_target_ctrl_units[3]),
                                      b=float(sys_mot_vec_target_ctrl_units[4]), c=float(sys_mot_vec_target_ctrl_units[5]),
                                      u=float(sys_mot_vec_target_ctrl_units[6]), v=float(sys_mot_vec_target_ctrl_units[7]),
                                      feedrate=feedrate, rapid=False)

        self.state.queued.motor_state_ctrl_units = sys_mot_vec_target_ctrl_units
        self.g_code_generated.emit(g_code)
        # print(f"write_g_code: {g_code}")

        return True

    def sys_motor_move_8d(self, mot_vec: np.ndarray, time: float | None = None, speed: float | None = None) -> bool:
        """ Move 8-joint
        :param mot_vec: 8-vector, radians and meters
        :param time: Motion time in seconds
        :param speed: Motion speed in rad/s
        """
        mot_vec_manipulator_rad = mot_vec[:config.N_REV_JNT]
        mot_vec_linear_axis_m = mot_vec[config.N_REV_JNT:config.N_JNT]

        mot_vec_man = self._manipulator.request_motor_move(mot_vec_manipulator_rad)
        if mot_vec_man is None:
            return False
        mot_vec_lin = self._linear_axis.request_joint_move(mot_vec_linear_axis_m)
        if mot_vec_lin is None:
            return False

        # Send to serial
        self.sys_queue_move(mot_vec_manipulator=mot_vec_man, mot_vec_linear_axis=mot_vec_lin, time=time, speed=speed)

        return True

    def sys_joint_move_8d(self, jnt_vec: np.ndarray, time: float | None = None, speed: float | None = None) -> bool:
        """ Move 8-joint
        :param jnt_vec: 8-vector, radians and meters
        :param time: Motion time in seconds
        :param speed: Motion speed in rad/s
        """
        mot_vec_manipulator_rad = jnt_vec[:config.N_REV_JNT]
        mot_vec_linear_axis_m = jnt_vec[config.N_REV_JNT:config.N_JNT]

        mot_vec_man = self._manipulator.request_joint_move(mot_vec_manipulator_rad)
        if mot_vec_man is None:
            return False
        mot_vec_lin = self._linear_axis.request_joint_move(mot_vec_linear_axis_m)
        if mot_vec_lin is None:
            return False

        # Send to serial
        self.sys_queue_move(mot_vec_manipulator=mot_vec_man, mot_vec_linear_axis=mot_vec_lin, time=time, speed=speed)

        return True

    def move_cartesian_6d(self, target_pose: Pose, time_seconds: float) -> bool:
        """ Motor space interpolated motion to given posture.
        :param target_pose:
        :param time_seconds:
        :return:
        """
        mot_vec = self._manipulator.request_cartesian_move(target_pose=target_pose)
        if mot_vec is None:
            return False

        self.sys_queue_move(mot_vec_manipulator=mot_vec, time=time_seconds)
        return True

    def move_cartesian_8d(self, target_pose: Pose, time_seconds: Optional[float] = None) -> Optional[np.ndarray]:
        """ Moves to cartesian space using 7 axis.
        :param target_pose: Target pose in world frame.
        :param time_seconds: Motion time in seconds.
        :return: True if motion accepted.
        """
        current_base_pose = self.state.queued.linear_axis.pose

        # For maximum manipulability, the target is at this distance from manipulator base.
        distance_desired = np.float64(np.sqrt(2)*config.CHAR_LEN)

        # Target XYZ coordinates relative to robot base at zero linear axis joints.
        target_wrt_zero_base: np.ndarray = target_pose.position - config.BASE_OFFSET

        # Y-distance less than ideal distance -> two solutions.
        if target_wrt_zero_base[1] < distance_desired:
            discriminant: np.float64 = np.sqrt(distance_desired ** 2 - target_wrt_zero_base[1] ** 2, dtype=np.float64)

            # Base x-positions in world frame, meters. Choose closer to previous solution.
            x_solutions = [target_pose.position[0] + discriminant, target_pose.position[0] - discriminant]
            x_solutions.sort(key=lambda new_x: np.abs(new_x - current_base_pose.position[0]))
            x = x_solutions[0]

        else:  # One solution
            x = target_pose.position[0]
            x = np.clip(x, 0.001*config.JOINT_LINEAR_LIMITS["JL1_MIN"], 0.001*config.JOINT_LINEAR_LIMITS["JL1_MAX"])

        # Compute pose of manipulator base.
        new_base_offset: np.ndarray = config.BASE_OFFSET.copy()
        new_base_offset[0] = x
        new_base_pose: Pose = Pose.from_position(new_base_offset)

        # Target
        manipulator_target: Pose = target_pose.relative_to(new_base_pose)
        # print(f"Tool w.r.t. world: {target_pose}")
        # print(f"Tool w.r.t. current base: {manipulator_target}")
        linear_axis_target: np.ndarray = np.array([x - config.BASE_OFFSET[0]])  # Absolute position for linear joint in meters.

        # Ensure motion is executable.
        mot_vec_manipulator: np.ndarray = self._manipulator.request_cartesian_move(manipulator_target)
        mot_vec_linear_axis: np.ndarray = self._linear_axis.request_joint_move(linear_axis_target)
        if mot_vec_manipulator is None or mot_vec_linear_axis is None:
            return None

        mot_vec: np.ndarray = np.zeros(8, dtype=np.float64)
        mot_vec[:config.N_REV_JNT] = mot_vec_manipulator
        mot_vec[config.N_REV_JNT:config.N_JNT] = mot_vec_linear_axis
        return mot_vec

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
            self.sys_queue_move(mot_vec_manipulator=vec, time=segment_time)

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
            self.sys_queue_move(mot_vec_manipulator=vec, time=segment_time)

        return True

    def execute_planned(self):
        """ Execute state found in planned state.
        """
        mot_vec: np.ndarray = self.state.planned.motor_state
        self.sys_motor_move_8d(mot_vec, time=10)

    def reset(self):
        self._manipulator.reset()

    def toggle_feed_hold(self):
        if self.state.controller == ControllerState.HOLD:
            self.g_code_generated.emit(self._gc_writer.cycle_start())
        elif self.state.controller == ControllerState.IDLE:
            self.g_code_generated.emit(self._gc_writer.cycle_start())
        elif self.state.controller == ControllerState.CYCLE:
            self.g_code_generated.emit(self._gc_writer.feed_hold())

    def update_status(self, status: str, mot_list: list, delta_t: float):
        """ Update system status and return by dictionary.
        @param status: str, controller status.
        @param mot_list: List of 8 floats, motor positions as degrees.
        @param delta_t: Time in seconds since last update.
        """
        match status:
            case "Idle":
                self.state.controller = ControllerState.IDLE
            case "Run":
                self.state.controller = ControllerState.CYCLE
            case "Hold":
                self.state.controller =  ControllerState.HOLD
            case "Home":
                self.state.controller = ControllerState.HOMING
            case "Alarm":
                self.state.controller = ControllerState.ALARM
            case "Check":
                self.state.controller = ControllerState.CHECK
            case "Door":
                self.state.controller = ControllerState.SAFETY_DOOR

        # Update MCU states with real motor values reported by controller.
        mot_vec_ctrl_units = np.array(mot_list)  # Degrees and millimeters
        self.state.mcu.motor_state_ctrl_units = mot_vec_ctrl_units
        jnt_vec_ctrl_units = self.state.mcu.joint_state_ctrl_units

        # Tool pose.
        tool_wrt_base: Pose = self.state.mcu.pose_tool_wrt_base
        tool_wrt_world: Pose = self.state.mcu.pose_tool_wrt_world

        # Condition
        # TODO: Call system state instead.
        cond_world = self._manipulator.state.mcu.condition(np.eye(3))
        cond_base = self._manipulator.state.mcu.condition(np.eye(3))
        cond_tool = self._manipulator.state.mcu.condition(tool_wrt_base.rot_mat)

        # TODO: Clean this mess.
        alpha = 0.95
        delta_x = np.linalg.norm(tool_wrt_base.position - self.previous_pose.position)
        speed_linear: float = alpha * utils.m_s2mm_min(delta_x / delta_t) + (1 - alpha) * self.speed_linear_prev
        speed_angular: float = alpha * utils.rad_sec2deg_min(abs(tool_wrt_base.quaternion.angle(self.previous_pose.quaternion)) / delta_t) + (1 - alpha) * self.speed_angular_prev
        self.speed_linear_prev, self.speed_angular_prev = speed_linear, speed_angular

        self.previous_pose = tool_wrt_base

        state = {
            'status': status,
            'mot_coords_deg': np.array(mot_list),
            'jnt_coords_deg': jnt_vec_ctrl_units,
            'ops_coords_base': tool_wrt_base,
            'ops_coords_world': tool_wrt_world,
            'link_poses': self._manipulator.state.mcu.link_poses,
            'spring_poses': self._manipulator.state.mcu.spring_poses,
            'counter_weight_pose': self._manipulator.state.mcu.counter_weight_pose,
            'condition_world': cond_world,
            'condition_base': cond_base,
            'condition_tool': cond_tool,
            'speed_linear': speed_linear,
            'speed_angular': speed_angular,
        }
        return state