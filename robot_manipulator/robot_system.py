""" Wrapper class for Manipulator

Author: Rainer Meyer, rot.meyer494@gmail.com
"""
from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot
import numpy as np
from typing import Optional, Tuple
from scipy import optimize

from robot_manipulator.kinematics.anthropomorphic_spherical_wrist import IkSolution
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

    def _sys_queue_move(self, mot_vec_manipulator: Optional[np.ndarray] = None,
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
        # print(f"write_g_code: {g_code}")  # Debug.

        return True

    def _request_cartesian_8d(self, target_pose: Pose) -> Optional[np.ndarray]:
        """ Solve motor coordinates for reaching a target in world space.
        :param target_pose: Target pose in world frame.
        :return: 8-vector of motor coordinates or None if target unreachable.
        """
        current_base_x = self.state.queued.linear_axis.pose.position[0]

        # Preferred distance from manipulator base to target to maximize manipulability.
        distance_desired = np.sqrt(2) * config.LINK_CHARACTERISTIC_LENGTH

        # Linear rail limits in world coordinates.
        x_min = 0.001 * config.JOINT_LINEAR_LIMITS["JL1_MIN"] + config.BASE_OFFSET[0]
        x_max = 0.001 * config.JOINT_LINEAR_LIMITS["JL1_MAX"] + config.BASE_OFFSET[0]

        # Target relative to base offset (ignoring rail x displacement).
        target_position: np.ndarray = target_pose.position
        base_offset: np.ndarray = config.BASE_OFFSET.copy()

        # Objective function for the scalar optimizer.
        def rail_objective(x_rail: float) -> float:
            """
            :param x_rail:
            :return:
            """
            # Distance from rail-positioned base to target in XY plane.
            dx = target_position[0] - x_rail
            dy = target_position[1] - base_offset[1]
            dz = target_position[2] - base_offset[2]
            dist_3d = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

            # Cost 1: Deviation from optimal manipulability distance.
            w_manip = 1.0
            cost_manip = w_manip * (dist_3d - distance_desired) ** 2

            # Cost 2: Preference for minimal rail motion.
            w_smooth = 0.2
            cost_smooth = w_smooth * (x_rail - current_base_x) ** 2

            return cost_manip + cost_smooth

        # Solve bounded 1D optimization for linear rail position within limits.
        res = optimize.minimize_scalar(rail_objective, bounds=(x_min, x_max), method='bounded')
        x_optimal = res.x

        # Compute target pose relative to updated base position.
        new_base_offset = config.BASE_OFFSET.copy()
        new_base_offset[0] = x_optimal
        new_base_pose = Pose.from_position(new_base_offset)
        current_base_x = new_base_pose.position[0]
        manipulator_target = target_pose.relative_to(new_base_pose)
        linear_axis_target = np.array([x_optimal - config.BASE_OFFSET[0]])

        # Request motor coordinates from manipulator and linear rail.
        mot_vec_manipulator = self._manipulator.request_cartesian_move(manipulator_target)
        mot_vec_linear_axis = self._linear_axis.request_joint_move(linear_axis_target)

        if mot_vec_manipulator is None or mot_vec_linear_axis is None:
            return None

        # Solution found.
        mot_vec = np.zeros(8, dtype=np.float64)
        mot_vec[:config.N_REV_JNT] = mot_vec_manipulator
        mot_vec[config.N_REV_JNT:config.N_JNT] = mot_vec_linear_axis

        return mot_vec

    # =============================================================================================
    # ====================================== Motion commands ======================================
    # =============================================================================================

    # ?====================================== 7-axis motion =======================================
    def move_motor_8d(self, mot_vec: np.ndarray, time: float | None = None, speed: float | None = None) -> bool:
        """ Move 8-joint
        :param mot_vec: 8-vector, radians and meters
        :param time: Motion time in seconds
        :param speed: Motion speed in rad/s
        :return: True if motion accepted and queued in serial.
        """
        # Request for motor coordinates.
        mot_vec_man = self._manipulator.request_motor_move(mot_vec[:config.N_REV_JNT])
        mot_vec_lin = self._linear_axis.request_joint_move(mot_vec[config.N_REV_JNT:config.N_JNT]) # joint=motor
        if mot_vec_man is None or mot_vec_lin is None:
            return False

        # Send to serial
        return self._sys_queue_move(mot_vec_manipulator=mot_vec_man, mot_vec_linear_axis=mot_vec_lin, time=time, speed=speed)

    def move_joint_8d(self, jnt_vec: np.ndarray, time: float | None = None, speed: float | None = None) -> bool:
        """ Move 8-joint
        :param jnt_vec: 8-vector, radians and meters
        :param time: Motion time in seconds
        :param speed: Motion speed in rad/s
        :return: True if motion accepted and queued in serial.
        """
        # Request for motor coordinates.
        mot_vec_man = self._manipulator.request_joint_move(jnt_vec[:config.N_REV_JNT])
        mot_vec_lin = self._linear_axis.request_joint_move(jnt_vec[config.N_REV_JNT:config.N_JNT])
        if mot_vec_man is None or mot_vec_lin is None:
            return False

        # Send to serial
        return self._sys_queue_move(mot_vec_manipulator=mot_vec_man, mot_vec_linear_axis=mot_vec_lin, time=time, speed=speed)

    def move_cartesian_8d(self, target_pose: Pose, time_seconds: Optional[float] = None) -> bool:
        mot_vec: np.ndarray = self._request_cartesian_8d(target_pose)
        if mot_vec is None:
            return False
        return self.move_motor_8d(mot_vec=mot_vec, time=time_seconds)

    # ===================================== Manipulator only  =====================================
    def move_cartesian_6d(self, target_pose: Pose, time_seconds: float, frame: str = "Base") -> bool:
        """ Motor space interpolated motion to given posture.
        :param target_pose:
        :param time_seconds:
        :param frame: World or Base
        :return: True if motion accepted and queued in serial.
        """
        if frame == "World":
            target_pose = target_pose.relative_to(self.state.queued.manipulator.base_pose)

        mot_vec = self._manipulator.request_cartesian_move(target_pose=target_pose)
        if mot_vec is None:
            return False

        return self._sys_queue_move(mot_vec_manipulator=mot_vec, time=time_seconds)

    def move_cartesian_linear_6d(self, target_pose: Pose, speed_linear: float = None, speed_angular: float = None,
                                      segment_length_m: float = 0.001, segment_size_rad: float = 0.0035) -> bool:
        """ Linear move in operational space  coordinates.
        :param target_pose: Target 6D-Pose in manipulator frame
        :param speed_linear: m/s, linear speed. By default, this is used.
        :param speed_angular: rad/s, rotational speed. Used if no linear speed is given.
        :param segment_length_m: m, Length of translational segment.
        :param segment_size_rad: rad, Size of rotational segment.
        :return: True if move was executed
        """
        ret: Optional[Tuple[np.ndarray, float]] = self._manipulator.request_cartesian_linear_move(
            target_pose, speed_linear, speed_angular, segment_length_m, segment_size_rad)

        if ret is None:
            return False

        mot_vecs, segment_time = ret  # Unpack motor vectors and segment time.
        for vec in mot_vecs:
            self._sys_queue_move(mot_vec_manipulator=vec, time=segment_time)

        return True

    def move_circle_6d(self, target_pose: Pose, speed_tangential: float, center: np.ndarray, normal: np.ndarray, total_angle: float, frame: str = "Base") -> bool:
        """ Move to target pose along an arc with center point at center, revolving around normal.
        :param target_pose: Pose object representing the end pose.
        :param speed_tangential: Tangential speed of arc motion, m/s
        :param center: Center point of the circular arc.
        :param normal: Normal vector of the circle plane.
        :param total_angle: Total sweep angle of the arc in radians.
        :param frame: Frame in which target is represented. World or Base.
        :return: True if move was executed
        """

        current_jnt_vec: np.ndarray = self.state.queued.manipulator.joint_state
        current_pose: Pose = self.state.queued.manipulator.ops_state

        if current_pose.is_close(target_pose):
            return False

        # Convert to manipulator base frame
        if frame == "World":
            target_pose = target_pose.relative_to(self.state.queued.manipulator.base_pose)
            center -= self.state.queued.manipulator.base_pose.position

        # Compute movement time (seconds)
        N_SEGMENT_FULL_REV: int = 40
        arc_length: float = total_angle * (np.linalg.norm(target_pose.position - center))  # meters.
        n_segments: int = int(np.ceil(N_SEGMENT_FULL_REV * total_angle * 0.5 / np.pi))
        move_time: float = arc_length / speed_tangential  # Seconds.
        segment_time = move_time / n_segments  # Seconds.

        mot_vecs = np.zeros((n_segments, config.N_REV_JNT))
        for idx in range(n_segments):
            t = (1+idx) / n_segments  # Interpolation parameter in range [0, 1]
            interp_pose: Pose = current_pose.arc_interpolate(target_pose, center, normal, total_angle, t)
            ik_sol: IkSolution = self._manipulator.kinematics.inverse(interp_pose, prev_jnt_vec=current_jnt_vec)
            if not ik_sol.success:
                print(f"move_ops_lin: IK fail {ik_sol.joint_limits} {ik_sol.singularity}")
                return False
            interp_jnt_vec = ik_sol.joint_solution
            current_jnt_vec = interp_jnt_vec.copy()

            # Propagate motion command forwards
            mot_vec = self._manipulator.request_joint_move(interp_jnt_vec)
            if mot_vec is None:
                return False

            mot_vecs[idx] = mot_vec

        for vec in mot_vecs:
            self._sys_queue_move(mot_vec_manipulator=vec, time=segment_time)

        return True

    def translate_tool_6d(self, direction_vec: tuple[int, int, int], distance: float, speed: float, frame: str) -> bool:
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
            self._sys_queue_move(mot_vec_manipulator=vec, time=segment_time)

        return True

    def rotate_tool_6d(self, direction_vec: tuple[int, int, int], angle: float, speed: float, frame: str) -> bool:
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
            self._sys_queue_move(mot_vec_manipulator=vec, time=segment_time)

        return True

    # =============================================================================================
    # =========================================== Slots ===========================================
    # =============================================================================================
    @pyqtSlot()
    def execute_planned(self):
        """ Execute state found in planned state.
        """
        mot_vec: np.ndarray = self.state.planned.motor_state
        self.move_motor_8d(mot_vec, time=10)

    @pyqtSlot()
    def revert_planned(self):
        self.state.planned.motor_state_ctrl_units = self.state.mcu.motor_state_ctrl_units

    @pyqtSlot()
    def reset(self):
        self._manipulator.reset()

    @pyqtSlot()
    def plan_8d(self) -> None:
        pose_world: Pose = self._manipulator.state.planner.link_nodes["PenHolder"].pose  # World frame

        mot_vec: np.ndarray = self._request_cartesian_8d(pose_world)
        if mot_vec is None:
            self.send_terminal.emit(f"Failed to compute joint solution for planned pose.")
            return

        self.state.planned.motor_state = mot_vec

    @pyqtSlot()
    def toggle_feed_hold(self):
        if self.state.controller == ControllerState.HOLD:
            self.g_code_generated.emit(self._gc_writer.cycle_start())
        elif self.state.controller == ControllerState.IDLE:
            self.g_code_generated.emit(self._gc_writer.cycle_start())
        elif self.state.controller == ControllerState.CYCLE:
            self.g_code_generated.emit(self._gc_writer.feed_hold())

    @pyqtSlot()
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