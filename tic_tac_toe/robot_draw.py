import numpy as np
from robot_manipulator.robot_system import RobotSystem
from robot_math.pose import Pose


def machine_draw_x(robot_sys: RobotSystem, char_pose: Pose, speed: float, size: float = 0.030) -> None:
    """ Draw the character 'X' to o_pose with given size and speed.
    :param robot_sys: Robot driver.
    :param char_pose: Pose of character in world frame. Lies on a plane normal to pose frames z-axis.
    :param speed: Linear speed of drawing motion.
    :param size: Height and width of 'X'.
    :return: None
    """
    z_offset: np.float64 = np.float64(-0.003)  # Draw the X slightly below the table for pen preload.
    sqrt2size: np.float64 = np.sqrt(2, dtype=np.float64) * size
    lead_in_out_time: float = 0.5  # Seconds
    lead_in_height: float = 0.015
    loiter_move_time: float = 3.  # Seconds.

    loiter_world: Pose = Pose.from_position(np.array([0.605, 0.201, 0.064]))
    # Corner numbering.
    #  4 \  / 1
    #     \/
    #     /\
    #  3 /  \ 2
    # corner_1: Pose = char_pose.offset(np.array([sqrt2size, sqrt2size, z_offset], dtype=np.float64))
    # corner_2: Pose = char_pose.offset(np.array([sqrt2size, -sqrt2size, z_offset], dtype=np.float64))
    # corner_3: Pose = char_pose.offset(np.array([-sqrt2size, -sqrt2size, z_offset], dtype=np.float64))
    # corner_4: Pose = char_pose.offset(np.array([-sqrt2size, sqrt2size, z_offset], dtype=np.float64))
    corner_1: Pose = Pose.from_rot_mat(pos=char_pose + np.array([sqrt2size, sqrt2size, z_offset]), R=rot_mat)
    corner_2: Pose = Pose.from_rot_mat(pos=char_pose + np.array([sqrt2size, -sqrt2size, z_offset]), R=rot_mat)
    corner_3: Pose = Pose.from_rot_mat(pos=char_pose + np.array([-sqrt2size, -sqrt2size, z_offset]), R=rot_mat)
    corner_4: Pose = Pose.from_rot_mat(pos=char_pose + np.array([-sqrt2size, sqrt2size, z_offset]), R=rot_mat)

    robot_sys.move_cartesian_8d(char_pose.offset(np.array([0., 0., 0.030])), time_seconds=loiter_move_time)
    # First line.
    robot_sys.move_cartesian_6d(corner_1.offset(np.array([0., 0., lead_in_height])), time_seconds=lead_in_out_time)  # Lead-in point.
    robot_sys.move_cartesian_linear_6d(corner_1, speed_linear=speed)  # Corner 1.
    robot_sys.move_cartesian_linear_6d(corner_3, speed_linear=speed)  # Corner 3.
    robot_sys.move_cartesian_6d(corner_3.offset(np.array([0., 0., lead_in_height])), time_seconds=lead_in_out_time)  # Lead-out point.
    # Second line.
    robot_sys.move_cartesian_6d(corner_2.offset(np.array([0., 0., lead_in_height])), time_seconds=lead_in_out_time)  # Lead-in point.
    robot_sys.move_cartesian_linear_6d(corner_2, speed_linear=speed)  # Corner 2.
    robot_sys.move_cartesian_linear_6d(corner_4, speed_linear=speed)  # Corner 4.
    robot_sys.move_cartesian_6d(corner_4.offset(np.array([0., 0., lead_in_height])), time_seconds=lead_in_out_time)  # Lead-out point.

    robot_sys.move_cartesian_8d(loiter_world, time_seconds=loiter_move_time)  # Move to loiter.

    # # X's corners in
    # corner_offsets_board: np.ndarray = np.zeros((3, 4))
    # corner_offsets_board[:, 0] = np.array([sqrt2size, sqrt2size, z_offset])
    # corner_offsets_board[:, 0] = np.array([sqrt2size, -sqrt2size, z_offset])
    # corner_offsets_board[:, 0] = np.array([-sqrt2size, -sqrt2size, z_offset])
    # corner_offsets_board[:, 0] = np.array([-sqrt2size, sqrt2size, z_offset])
    # corner_offsets_world = char_pose.rot_mat @ corner_offsets_board
    #
    # corner1: np.ndarray = char_pose.position + corner_offsets_world[:, 0]
    # corner2: np.ndarray = char_pose.position + corner_offsets_world[:, 1]
    # corner3: np.ndarray = char_pose.position + corner_offsets_world[:, 2]
    # corner4: np.ndarray = char_pose.position + corner_offsets_world[:, 3]

def machine_draw_o(robot_sys: RobotSystem, char_pose: Pose, speed: float, size: float = 0.030) -> None:
    """ Draw the character 'O' to o_pose with given size and speed.
    :param robot_sys: Robot driver.
    :param char_pose: Pose of character. Lies on a plane normal to pose frames z-axis.
    :param speed: Tangential speed of motion.
    :param size: Diameter of 'O'.
    :return: None
    """

    z_offset: np.float64 = np.float64(-0.003)  # Draw the X slightly below the table for pen preload.
    radius: np.float64 = np.float64(size / 2)
    lead_in_out_time: float = 0.5  # Seconds
    loiter_move_time: float = 3.  # Seconds.

    above_start_end: Pose = char_pose.offset(np.array([radius, 0., 0.020], dtype=np.float64))
    circle_start_end: Pose = char_pose.offset(np.array([radius, 0, z_offset], dtype=np.float64))
    circle_center: Pose = char_pose.offset(np.asarray([0., 0., z_offset], dtype=np.float64))
    loiter_world: Pose = Pose.from_position(np.array([0.605, 0.201, 0.064]))

    robot_sys.move_cartesian_8d(above_start_end, time_seconds=loiter_move_time)
    robot_sys.move_cartesian_6d(circle_start_end, time_seconds=lead_in_out_time, frame="World")
    robot_sys.move_circle_6d(target_pose=circle_start_end, speed_tangential=speed, center=circle_center.position, normal=np.array([0., 0., 1.]), total_angle=2*np.pi, frame="World")
    robot_sys.move_cartesian_6d(above_start_end, time_seconds=lead_in_out_time, frame="World")
    robot_sys.move_cartesian_8d(loiter_world, time_seconds=loiter_move_time)
