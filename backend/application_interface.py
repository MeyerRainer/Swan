""" Interface connecting all subsystems.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from backend.robot_system import RobotSystem
from backend.camera_manager import CameraManager
from backend.vision_manager import VisionManager
from backend.gc_serial import GCSerial
from backend import utils

from robot_program import parser
from robot_program.executor import InstructionExecutor

from PyQt6.QtCore import QObject
import numpy as np
import serial

from robot_program.program import Program


class ApplicationInterface(QObject):

    def __init__(self, main_window):

        super().__init__()

        self.main_window = main_window

        # Components
        self.serial = GCSerial()
        self.robot_sys = RobotSystem()
        self.camera = CameraManager()
        self.vision = VisionManager()
        self.program_parser = parser.ProgramParser()
        self.program = None
        self.executor = InstructionExecutor(parent=self.robot_sys)

        # Widget panels
        self.toolbar = self.main_window.toolbar
        self.control_panel = self.main_window.control
        self.dro_panel = self.main_window.dro
        self.teach_panel = self.main_window.teach_interface
        self.program_panel = self.main_window.program_control
        self.terminal_panel = self.main_window.terminal.widget()

        # Viewport
        self._view_3d = self.main_window.view_3d
        self._view_camera = self.main_window.view_camera

        # Initialize
        self.connect_signals()

    def get_program(self) -> Program:
        return self.program

    def execute_program(self):
        if self.program is None:
            self.terminal_panel.write("No program loaded.")
            return

        self.executor.execute(self.program)

    def connect_signals(self):
        # =========================================== Toolbar ============================================
        self.toolbar.refresh_button.clicked.connect(self.refresh_ports)
        self.toolbar.connect_button.clicked.connect(self.toggle_connection)

        # ======================================== Control panel =========================================
        self.control_panel.home_button.clicked.connect(self.home)
        # Translation
        self.control_panel.x_plus.clicked.connect(lambda: self.robot_sys.translate_tool((1, 0, 0), self.control_panel.increment_linear.value() / 1000, utils.mm_min2m_s(self.control_panel.speed_linear.value()), self.control_panel.frame_select_position.currentText()))
        self.control_panel.x_minus.clicked.connect(lambda: self.robot_sys.translate_tool((-1, 0, 0), self.control_panel.increment_linear.value() / 1000, utils.mm_min2m_s(self.control_panel.speed_linear.value()), self.control_panel.frame_select_position.currentText()))
        self.control_panel.y_plus.clicked.connect(lambda: self.robot_sys.translate_tool((0, 1, 0), self.control_panel.increment_linear.value() / 1000, utils.mm_min2m_s(self.control_panel.speed_linear.value()), self.control_panel.frame_select_position.currentText()))
        self.control_panel.y_minus.clicked.connect(lambda: self.robot_sys.translate_tool((0, -1, 0), self.control_panel.increment_linear.value() / 1000, utils.mm_min2m_s(self.control_panel.speed_linear.value()), self.control_panel.frame_select_position.currentText()))
        self.control_panel.z_plus.clicked.connect(lambda: self.robot_sys.translate_tool((0, 0, 1), self.control_panel.increment_linear.value() / 1000, utils.mm_min2m_s(self.control_panel.speed_linear.value()), self.control_panel.frame_select_position.currentText()))
        self.control_panel.z_minus.clicked.connect(lambda: self.robot_sys.translate_tool((0, 0, -1), self.control_panel.increment_linear.value() / 1000, utils.mm_min2m_s(self.control_panel.speed_linear.value()), self.control_panel.frame_select_position.currentText()))
        self.control_panel.x_plus_y_plus.clicked.connect(lambda: self.robot_sys.translate_tool((1, 1, 0), self.control_panel.increment_linear.value() / 1000, utils.mm_min2m_s(self.control_panel.speed_linear.value()), self.control_panel.frame_select_position.currentText()))
        self.control_panel.x_plus_y_minus.clicked.connect(lambda: self.robot_sys.translate_tool((1, -1, 0), self.control_panel.increment_linear.value() / 1000, utils.mm_min2m_s(self.control_panel.speed_linear.value()), self.control_panel.frame_select_position.currentText()))
        self.control_panel.x_minus_y_minus.clicked.connect(lambda: self.robot_sys.translate_tool((-1, -1, 0), self.control_panel.increment_linear.value() / 1000, utils.mm_min2m_s(self.control_panel.speed_linear.value()), self.control_panel.frame_select_position.currentText()))
        self.control_panel.x_minus_y_plus.clicked.connect(lambda: self.robot_sys.translate_tool((-1, 1, 0), self.control_panel.increment_linear.value() / 1000, utils.mm_min2m_s(self.control_panel.speed_linear.value()), self.control_panel.frame_select_position.currentText()))
        # Rotation
        self.control_panel.rx_plus.clicked.connect(lambda: self.robot_sys.rotate_tool((1, 0, 0), np.deg2rad(self.control_panel.increment_angular.value()), utils.deg_min2rad_sec(self.control_panel.speed_angular.value()), self.control_panel.frame_select_orientation.currentText()))
        self.control_panel.rx_minus.clicked.connect(lambda: self.robot_sys.rotate_tool((-1, 0, 0), np.deg2rad(self.control_panel.increment_angular.value()), utils.deg_min2rad_sec(self.control_panel.speed_angular.value()), self.control_panel.frame_select_orientation.currentText()))
        self.control_panel.ry_plus.clicked.connect(lambda: self.robot_sys.rotate_tool((0, 1, 0), np.deg2rad(self.control_panel.increment_angular.value()), utils.deg_min2rad_sec(self.control_panel.speed_angular.value()), self.control_panel.frame_select_orientation.currentText()))
        self.control_panel.ry_minus.clicked.connect(lambda: self.robot_sys.rotate_tool((0, -1, 0), np.deg2rad(self.control_panel.increment_angular.value()), utils.deg_min2rad_sec(self.control_panel.speed_angular.value()), self.control_panel.frame_select_orientation.currentText()))
        self.control_panel.rz_plus.clicked.connect(lambda: self.robot_sys.rotate_tool((0, 0, 1), np.deg2rad(self.control_panel.increment_angular.value()), utils.deg_min2rad_sec(self.control_panel.speed_angular.value()), self.control_panel.frame_select_orientation.currentText()))
        self.control_panel.rz_minus.clicked.connect(lambda: self.robot_sys.rotate_tool((0, 0, -1), np.deg2rad(self.control_panel.increment_angular.value()), utils.deg_min2rad_sec(self.control_panel.speed_angular.value()), self.control_panel.frame_select_orientation.currentText()))
        # Manipulator joints
        self.control_panel.j1_slider.connect_target(self.read_sliders)
        self.control_panel.j2_slider.connect_target(self.read_sliders)
        self.control_panel.j3_slider.connect_target(self.read_sliders)
        self.control_panel.j4_slider.connect_target(self.read_sliders)
        self.control_panel.j5_slider.connect_target(self.read_sliders)
        self.control_panel.j6_slider.connect_target(self.read_sliders)
        # self.control_panel.j7_slider.connect_target(self.read_sliders)
        self.control_panel.jl1_slider.connect_target(self.read_sliders)
        # self.control_panel.jl2_slider.connect_target(self.read_sliders)

        # ========================================= Manipulator ==========================================
        self.robot_sys.g_code_generated.connect(self.serial.send)
        self.robot_sys.send_terminal.connect(self.terminal_panel.write)

        # =========================================== Serial =============================================
        self.serial.connected.connect(self.toolbar.on_connect)
        self.serial.disconnected.connect(self.toolbar.on_disconnect)
        # self.serial.status_received.connect(self.on_status_update)
        # self.serial.error_received.connect(self.terminal.write)
        # self.serial.alarm_received.connect(self.terminal.write)
        self.serial.line_received.connect(self.process_line)

        # =========================================== Terminal ===========================================
        self.terminal_panel.command_signal.connect(self.serial.send)

        # ========================================== Viewport ============================================
        self.main_window.viewport_tabs.currentChanged.connect(self.on_viewport_tab_change)

        # =========================================== Camera =============================================
        self.camera.frame_received.connect(self.vision.process_frame)
        self.camera.frame_received.connect(self._view_camera.show_frame)
        self.camera.error.connect(self.terminal_panel.write)
        # self.vision.processed_frame.connect(self._view_camera.show_frame)

        # ======================================== Program panel =========================================
        self.program_panel.run_button.clicked.connect(self.execute_program)
        self.program_panel.directory_changed.connect(self.main_window.save_last_directory)
        self.program_panel.file_loaded.connect(self.load_program)
        self.program_panel.run_button.clicked.connect(self.execute_program)

    def load_program(self, file: str, extension: str):
        self.program = self.program_parser.parse(file, extension)
        self.terminal_panel.write(f"Program loaded: {self.program.name}")

    def refresh_ports(self):
        self.toolbar.port_combo.clear()
        ports: list = self.serial.scan_ports()
        if not ports:
            self.terminal_panel.write("No ports found")
            return
        else:
            self.toolbar.port_combo.addItems(ports)
            self.terminal_panel.write("Ports refreshed")

    def toggle_connection(self):
        # Disconnect
        if self.serial.is_connected:
            self.serial.disconnect()
            # self.camera.stop()
            self.terminal_panel.write("Disconnected")
            self.toolbar.on_disconnect()
            return

        default = self.toolbar.port_combo.currentText()
        ports = [default] if default != '' else  []

        baud = int(self.toolbar.baud_combo.currentText())

        autoscan = True
        if autoscan:
            ports.extend(self.serial.scan_ports())

        if not ports:
            self.terminal_panel.write("No ports found")
            return

        for port in ports:
            try:
                if self.serial.connect(port, baud):
                    # Connected
                    self.terminal_panel.write(f"Connected. Manipulator reset.")
                    self.toolbar.on_connect()
                    self.robot_sys.reset()
                    return

            except serial.SerialException as e:
                self.terminal_panel.write(f"Failed to connect {port}: {e}")

        self.terminal_panel.write(f"Failed to connect")

    def process_line(self, line: str):
        if line.startswith('<'):
            self.on_status_update(line)

    def on_status_update(self, grbl_status_str):
        """ Upon receiving status from GRBL, update complete system status
        @param grbl_status_str: str, GRBL-styled status message
        """
        # Update robot state
        status, m_pos, w_pos = utils.parse_grbl_status(grbl_status_str)
        self.toolbar.update_status(status)

        # Update manipulator state
        status_dict: dict = self.robot_sys.update_status(m_pos)

        # Update digital readout
        self.dro_panel.update_status(status_dict)

        # Update viewport
        self._view_3d.update_status(status_dict)

        # Update sliders
        self.control_panel.update_joint_sliders(status_dict['jnt_coords_deg'])

    def on_viewport_tab_change(self, index):

        widget = self.main_window.viewport_tabs.widget(index)

        if widget is self._view_camera:
            self.camera.connect()
        else:
            self.camera.disconnect()

    def read_sliders(self):
        # Revolute
        j1 = np.deg2rad(self.control_panel.j1_slider.value())
        j2 = np.deg2rad(self.control_panel.j2_slider.value())
        j3 = np.deg2rad(self.control_panel.j3_slider.value())
        j4 = np.deg2rad(self.control_panel.j4_slider.value())
        j5 = np.deg2rad(self.control_panel.j5_slider.value())
        j6 = np.deg2rad(self.control_panel.j6_slider.value())
        # Linear
        j7 = 0.001 * self.control_panel.jl1_slider.value()
        j8 = 0.001 * self.control_panel.jl2_slider.value()

        jnt_vec = np.array([j1, j2, j3, j4, j5, j6, j7, j8])
        speed = utils.deg_min2rad_sec(self.control_panel.speed_joint.value())

        self.robot_sys.move_jnt(jnt_vec, time=None, speed=speed)

    def home(self):
        self.robot_sys.move_jnt(np.zeros(8), time=None, speed=utils.deg_min2rad_sec(self.control_panel.speed_joint.value()))
