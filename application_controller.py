""" Interface connecting main_window with context.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
import utils

from typing import Tuple
from PyQt6.QtCore import QObject
import numpy as np


class ApplicationController(QObject):

    def __init__(self, main_window, application_context):

        super().__init__()

        self.main_window = main_window              # GUI
        self.app_context = application_context      # Application state

        # Initialize
        self.connect_signals()

    def connect_signals(self):

        # =========================================== Toolbar ============================================
        self.main_window.toolbar.refresh_button.clicked.connect(self.app_context.serial.refresh_ports)
        self.main_window.toolbar.connect_button.clicked.connect(self.app_context.serial.toggle_connection)
        self.main_window.toolbar.baud_combo.currentTextChanged.connect(self.app_context.serial.on_baud_receive)
        self.main_window.toolbar.port_combo.currentTextChanged.connect(self.app_context.serial.on_port_receive)

        # ======================================== Control panel =========================================
        self.main_window.control_panel.home_button.clicked.connect(self.home)
        # Translation
        self.main_window.control_panel.x_plus.clicked.connect(lambda: self.translate_robot_sys(direction=(1, 0, 0)))
        self.main_window.control_panel.x_minus.clicked.connect(lambda: self.translate_robot_sys(direction=(-1, 0, 0)))
        self.main_window.control_panel.y_plus.clicked.connect(lambda: self.translate_robot_sys(direction=(0, 1, 0)))
        self.main_window.control_panel.y_minus.clicked.connect(lambda: self.translate_robot_sys(direction=(0, -1, 0)))
        self.main_window.control_panel.z_plus.clicked.connect(lambda: self.translate_robot_sys(direction=(0, 0, 1)))
        self.main_window.control_panel.z_minus.clicked.connect(lambda: self.translate_robot_sys(direction=(0, 0, -1)))
        self.main_window.control_panel.x_plus_y_plus.clicked.connect(lambda: self.translate_robot_sys(direction=(1, 1, 0)))
        self.main_window.control_panel.x_plus_y_minus.clicked.connect(lambda: self.translate_robot_sys(direction=(1, -1, 0)))
        self.main_window.control_panel.x_minus_y_minus.clicked.connect(lambda: self.translate_robot_sys(direction=(-1, -1, 0)))
        self.main_window.control_panel.x_minus_y_plus.clicked.connect(lambda: self.translate_robot_sys(direction=(-1, 1, 0)))
        # Rotation
        self.main_window.control_panel.rx_plus.clicked.connect(lambda : self.rotate_robot_sys(direction=(1, 0, 0)))
        self.main_window.control_panel.rx_minus.clicked.connect(lambda: self.rotate_robot_sys(direction=(-1, 0, 0)))
        self.main_window.control_panel.ry_plus.clicked.connect(lambda: self.rotate_robot_sys(direction=(0, 1, 0)))
        self.main_window.control_panel.ry_minus.clicked.connect(lambda: self.rotate_robot_sys(direction=(0, -1, 0)))
        self.main_window.control_panel.rz_plus.clicked.connect(lambda: self.rotate_robot_sys(direction=(0, 0, 1)))
        self.main_window.control_panel.rz_minus.clicked.connect(lambda: self.rotate_robot_sys(direction=(0, 0, -1)))
        # Manipulator joints
        self.main_window.control_panel.j1_slider.connect_target(self.joint_move_robot_sys)
        self.main_window.control_panel.j2_slider.connect_target(self.joint_move_robot_sys)
        self.main_window.control_panel.j3_slider.connect_target(self.joint_move_robot_sys)
        self.main_window.control_panel.j4_slider.connect_target(self.joint_move_robot_sys)
        self.main_window.control_panel.j5_slider.connect_target(self.joint_move_robot_sys)
        self.main_window.control_panel.j6_slider.connect_target(self.joint_move_robot_sys)
        # self.main_window.control_panel.j7_slider.connect_target(self.read_sliders)
        self.main_window.control_panel.jl1_slider.connect_target(self.joint_move_robot_sys)
        # self.main_window.control_panel.jl2_slider.connect_target(self.read_sliders)

        # ========================================= Robot system =========================================
        self.app_context.robot_sys.send_terminal.connect(self.main_window.terminal.write)

        # =========================================== Serial =============================================
        self.app_context.serial.connected.connect(self.main_window.toolbar.on_connect)
        self.app_context.serial.disconnected.connect(self.main_window.toolbar.on_disconnect)
        self.app_context.serial.serial_connect.connect(self.main_window.toolbar.on_serial_toggle)
        self.app_context.serial.write_terminal.connect(self.main_window.terminal.write)
        self.app_context.serial.ports_refreshed.connect(self.main_window.toolbar.display_ports)
        self.app_context.serial.status_received.connect(self.on_status_update)
        # self.serial.error_received.connect(self.terminal.write)
        # self.serial.alarm_received.connect(self.terminal.write)

        # =========================================== Terminal ===========================================
        # self.terminal_panel.command_signal.connect(self.serial.send)  # Send to serial. First parse message.

        # ========================================== Viewport ============================================
        self.main_window.viewport_tabs.currentChanged.connect(self.on_viewport_tab_change)

        # =========================================== Camera =============================================
        self.app_context.camera.frame_received.connect(self.main_window.view_camera.show_frame)
        self.app_context.camera.error.connect(self.main_window.terminal.write)
        self.app_context.vision.processed_frame.connect(self.main_window.view_camera.show_frame)

        # ======================================== Program panel =========================================
        # self.main_window.program_panel.run_button.clicked.connect(self.app_context.execute_program)
        # self.main_window.program_panel.directory_changed.connect(self.main_window.save_last_directory)
        # self.main_window.program_panel.file_loaded.connect(self.load_program)
        # self.main_window.program_panel.run_button.clicked.connect(self.execute_program)

        # ========================================= Scene panel ==========================================
        # self.scene_tree.setModel(scene_tree_model)

    def on_viewport_tab_change(self, index):

        widget = self.main_window.viewport_tabs.widget(index)

        if widget is self.main_window.view_camera:
            self.app_context.camera.connect()
        else:
            self.app_context.camera.disconnect()

    def joint_move_robot_sys(self):
        # Revolute
        j1 = np.deg2rad(self.main_window.control_panel.j1_slider.value())
        j2 = np.deg2rad(self.main_window.control_panel.j2_slider.value())
        j3 = np.deg2rad(self.main_window.control_panel.j3_slider.value())
        j4 = np.deg2rad(self.main_window.control_panel.j4_slider.value())
        j5 = np.deg2rad(self.main_window.control_panel.j5_slider.value())
        j6 = np.deg2rad(self.main_window.control_panel.j6_slider.value())
        # Linear
        j7 = 0.001 * self.main_window.control_panel.jl1_slider.value()
        j8 = 0.001 * self.main_window.control_panel.jl2_slider.value()

        jnt_vec = np.array([j1, j2, j3, j4, j5, j6, j7, j8])
        speed = utils.deg_min2rad_sec(self.main_window.control_panel.speed_joint.value())

        self.app_context.robot_sys.move_jnt(jnt_vec, time=None, speed=speed)

    def translate_robot_sys(self, direction: Tuple[int, int, int]):
        distance: float = self.main_window.control_panel.increment_linear.value() / 1000       # m
        speed: float = utils.mm_min2m_s(self.main_window.control_panel.speed_linear.value())    # m/s
        frame: str = self.main_window.control_panel.frame_select_position.currentText()
        self.app_context.robot_sys.translate_tool(direction_vec=direction, distance=distance, speed=speed, frame=frame)

    def rotate_robot_sys(self, direction: Tuple[int, int, int]):
        frame: str = self.main_window.control_panel.frame_select_orientation.currentText()
        angle_increment: float = np.deg2rad(self.main_window.control_panel.increment_angular.value())
        speed: float = utils.deg_min2rad_sec(self.main_window.control_panel.speed_angular.value())
        self.app_context.robot_sys.rotate_tool(direction_vec=direction, angle=angle_increment, speed=speed, frame=frame)

    def home(self):
        self.app_context.robot_sys.move_jnt(np.zeros(8), time=None, speed=utils.deg_min2rad_sec(self.main_window.control_panel.speed_joint.value()))


    def on_status_update(self, grbl_status_str):
        """ Upon receiving status from GRBL, update complete system status
        @param grbl_status_str: str, GRBL-styled status message
        """
        # Update robot state
        status, m_pos, w_pos = utils.parse_grbl_status(grbl_status_str)
        self.main_window.toolbar.update_status(status)

        # Update manipulator state
        status_dict: dict = self.app_context.robot_sys.update_status(m_pos)

        # Update digital readout
        self.main_window.dro_panel.update_status(status_dict)

        # Update viewport
        self.main_window.view_3d.update_status(status_dict)

        # Update sliders
        self.main_window.control_panel.update_joint_sliders(status_dict['jnt_coords_deg'])
