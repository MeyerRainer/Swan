""" Application interface to combine Manipulator and GCSerial with MainWindow.
Author: Rainer Meyer, rot.meyer494@gmail.com
"""
from backend.manipulator_manager import ManipulatorManager
from backend.camera_manager import CameraManager
from backend.vision_manager import VisionManager
from backend.gc_serial import GCSerial
from backend import utils

from robot_program import parser
from robot_program.program import Program
from robot_program.instructions.instruction import Instruction
from robot_program.executor import InstructionExecutor

from PyQt6.QtCore import QObject
import numpy as np
import serial

from robot_program.program import Program


class ApplicationInterface(QObject):

    def __init__(self, main_window):

        super().__init__()

        self.serial = GCSerial()
        self.manipulator = ManipulatorManager()
        self._camera = CameraManager()
        self._vision = VisionManager()

        self._parser = parser.ProgramParser()
        self.program = None
        self.executor = InstructionExecutor(parent=self.manipulator)

        self._gui = main_window

        # Docks
        self._toolbar = self._gui.toolbar
        self._dro = self._gui.dro.widget()
        self._control = self._gui.control.widget()
        self._terminal = self._gui.terminal.widget()
        self._program_control = self._gui.program_control.widget()
        self._teach_interface = self._gui.teach_interface.widget()

        # Viewport
        self._view_3d = self._gui.view_3d
        self._view_camera = self._gui.view_camera

        # Initialize
        self.connect_gui()
        self.connect_signals()

    def get_program(self) -> Program:
        return self.program

    def execute_program(self):
        if self.program is None:
            self._terminal.write("No program loaded.")
            return

        self.executor.execute(self.program)

    def connect_gui(self):
        # GUI -> Backend
        # Toolbar
        self._toolbar.refresh_button.clicked.connect(self.refresh_ports)
        self._toolbar.connect_button.clicked.connect(self.toggle_connection)
        self._program_control.run_button.clicked.connect(self.execute_program)

        # ======================================== Control panel =========================================
        # Translation
        self._control.x_plus.clicked.connect(lambda: self.manipulator.translate_tool((1, 0, 0), self._control.increment_linear.value() / 1000, utils.mm_min2m_s(self._control.speed_linear.value()), self._control.frame_select_position.currentText()))
        self._control.x_minus.clicked.connect(lambda: self.manipulator.translate_tool((-1, 0, 0), self._control.increment_linear.value() / 1000, utils.mm_min2m_s(self._control.speed_linear.value()), self._control.frame_select_position.currentText()))
        self._control.y_plus.clicked.connect(lambda: self.manipulator.translate_tool((0, 1, 0), self._control.increment_linear.value() / 1000, utils.mm_min2m_s(self._control.speed_linear.value()), self._control.frame_select_position.currentText()))
        self._control.y_minus.clicked.connect(lambda: self.manipulator.translate_tool((0, -1, 0), self._control.increment_linear.value() / 1000, utils.mm_min2m_s(self._control.speed_linear.value()), self._control.frame_select_position.currentText()))
        self._control.z_plus.clicked.connect(lambda: self.manipulator.translate_tool((0, 0, 1), self._control.increment_linear.value() / 1000, utils.mm_min2m_s(self._control.speed_linear.value()), self._control.frame_select_position.currentText()))
        self._control.z_minus.clicked.connect(lambda: self.manipulator.translate_tool((0, 0, -1), self._control.increment_linear.value() / 1000, utils.mm_min2m_s(self._control.speed_linear.value()), self._control.frame_select_position.currentText()))
        self._control.x_plus_y_plus.clicked.connect(lambda: self.manipulator.translate_tool((1, 1, 0), self._control.increment_linear.value() / 1000, utils.mm_min2m_s(self._control.speed_linear.value()), self._control.frame_select_position.currentText()))
        self._control.x_plus_y_minus.clicked.connect(lambda: self.manipulator.translate_tool((1, -1, 0), self._control.increment_linear.value() / 1000, utils.mm_min2m_s(self._control.speed_linear.value()), self._control.frame_select_position.currentText()))
        self._control.x_minus_y_minus.clicked.connect(lambda: self.manipulator.translate_tool((-1, -1, 0), self._control.increment_linear.value() / 1000, utils.mm_min2m_s(self._control.speed_linear.value()), self._control.frame_select_position.currentText()))
        self._control.x_minus_y_plus.clicked.connect(lambda: self.manipulator.translate_tool((-1, 1, 0), self._control.increment_linear.value() / 1000, utils.mm_min2m_s(self._control.speed_linear.value()), self._control.frame_select_position.currentText()))
        # Rotation
        self._control.rx_plus.clicked.connect(lambda: self.manipulator.rotate_tool((1, 0, 0), np.deg2rad(self._control.increment_angular.value()), utils.deg_min2rad_sec(self._control.speed_angular.value()), self._control.frame_select_orientation.currentText()))
        self._control.rx_minus.clicked.connect(lambda: self.manipulator.rotate_tool((-1, 0, 0), np.deg2rad(self._control.increment_angular.value()), utils.deg_min2rad_sec(self._control.speed_angular.value()), self._control.frame_select_orientation.currentText()))
        self._control.ry_plus.clicked.connect(lambda: self.manipulator.rotate_tool((0, 1, 0), np.deg2rad(self._control.increment_angular.value()), utils.deg_min2rad_sec(self._control.speed_angular.value()), self._control.frame_select_orientation.currentText()))
        self._control.ry_minus.clicked.connect(lambda: self.manipulator.rotate_tool((0, -1, 0), np.deg2rad(self._control.increment_angular.value()), utils.deg_min2rad_sec(self._control.speed_angular.value()), self._control.frame_select_orientation.currentText()))
        self._control.rz_plus.clicked.connect(lambda: self.manipulator.rotate_tool((0, 0, 1), np.deg2rad(self._control.increment_angular.value()), utils.deg_min2rad_sec(self._control.speed_angular.value()), self._control.frame_select_orientation.currentText()))
        self._control.rz_minus.clicked.connect(lambda: self.manipulator.rotate_tool((0, 0, -1), np.deg2rad(self._control.increment_angular.value()), utils.deg_min2rad_sec(self._control.speed_angular.value()), self._control.frame_select_orientation.currentText()))
        # Manipulator joints
        self._control.j1_slider.connect_target(lambda val: self.manipulator.move_single_jnt_manipulator(0, angle=np.deg2rad(val), speed=utils.deg_min2rad_sec(self._control.speed_joint.value())))
        self._control.j2_slider.connect_target(lambda val: self.manipulator.move_single_jnt_manipulator(1, angle=np.deg2rad(val), speed=utils.deg_min2rad_sec(self._control.speed_joint.value())))
        self._control.j3_slider.connect_target(lambda val: self.manipulator.move_single_jnt_manipulator(2, angle=np.deg2rad(val), speed=utils.deg_min2rad_sec(self._control.speed_joint.value())))
        self._control.j4_slider.connect_target(lambda val: self.manipulator.move_single_jnt_manipulator(3, angle=np.deg2rad(val), speed=utils.deg_min2rad_sec(self._control.speed_joint.value())))
        self._control.j5_slider.connect_target(lambda val: self.manipulator.move_single_jnt_manipulator(4, angle=np.deg2rad(val), speed=utils.deg_min2rad_sec(self._control.speed_joint.value())))
        self._control.j6_slider.connect_target(lambda val: self.manipulator.move_single_jnt_manipulator(5, angle=np.deg2rad(val), speed=utils.deg_min2rad_sec(self._control.speed_joint.value())))
        # self._control.j7_slider.connect_target(lambda val: self.manipulator.move_single_jnt_manipulator(6, angle=np.deg2rad(val), speed=utils.deg_min2rad_sec(self._control.speed_joint.value())))
        # Linear axis joints
        self._control.jl1_slider.connect_target(lambda val: self.manipulator.move_single_jnt_linear_axis(0, distance=0.001*val, speed=utils.mm_min2m_s(self._control.speed_joint.value())))
        # self._control.jl2_slider.connect_target(lambda val: self.manipulator.move_single_jnt_linear_axis(0, distance=0.001*val, speed=utils.mm_min2m_s(self._control.speed_joint.value())))


    def connect_signals(self):
        # --- Backend -> GUI ---
        # Manipulator
        self.manipulator.g_code_generated.connect(self.serial.send)
        self.manipulator.send_terminal.connect(self._terminal.write)

        # Serial
        self.serial.connected.connect(self._toolbar.on_connect)
        self.serial.disconnected.connect(self._toolbar.on_disconnect)
        # self.serial.status_received.connect(self.on_status_update)
        # self.serial.error_received.connect(self.terminal.write)
        # self.serial.alarm_received.connect(self.terminal.write)
        self.serial.line_received.connect(self.process_line)

        # Terminal
        self._terminal.command_signal.connect(self.serial.send)

        # Viewport
        self._gui.viewport_tabs.currentChanged.connect(self.on_viewport_tab_change)

        # Camera
        self._camera.frame_received.connect(self._vision.process_frame)
        self._camera.frame_received.connect(self._view_camera.show_frame)
        self._camera.error.connect(self._terminal.write)
        # self._vision.processed_frame.connect(self._view_camera.show_frame)

        # Program panel
        self._program_control.directory_changed.connect(self._gui.save_last_directory)
        self._program_control.file_loaded.connect(self.load_program)
        self._program_control.run_button.clicked.connect(self.execute_program)

    def load_program(self, file: str, extension: str):
        self.program = self._parser.parse(file, extension)
        self._terminal.write(f"Program loaded: {self.program.name}")

    def refresh_ports(self):
        self._toolbar.port_combo.clear()
        ports: list = self.serial.scan_ports()
        if not ports:
            self._terminal.write("No ports found")
            return
        else:
            self._toolbar.port_combo.addItems(ports)
            self._terminal.write("Ports refreshed")

    def toggle_connection(self):
        # Disconnect
        if self.serial.is_connected:
            self.serial.disconnect()
            # self._camera.stop()
            self._terminal.write("Disconnected")
            self._toolbar.on_disconnect()
            return

        default = self._toolbar.port_combo.currentText()
        ports = [default] if default != '' else  []

        baud = int(self._toolbar.baud_combo.currentText())

        autoscan = True
        if autoscan:
            ports.extend(self.serial.scan_ports())

        if not ports:
            self._terminal.write("No ports found")
            return

        for port in ports:
            try:
                if self.serial.connect(port, baud):
                    # Connected
                    self._terminal.write(f"Connected. Manipulator reset.")
                    self._toolbar.on_connect()
                    self.manipulator.reset()
                    return

            except serial.SerialException as e:
                self._terminal.write(f"Failed to connect {port}: {e}")

        self._terminal.write(f"Failed to connect")

    def process_line(self, line: str):
        if line.startswith('<'):
            self.on_status_update(line)

    def on_status_update(self, grbl_status_str):
        """ Upon receiving status from GRBL, update complete system status
        @param grbl_status_str: str, GRBL-styled status message
        """
        # Update robot state
        status, m_pos, w_pos = utils.parse_grbl_status(grbl_status_str)
        self._toolbar.update_status(status)

        # Update manipulator state
        status_dict: dict = self.manipulator.update_status(m_pos)

        # Update digital readout
        self._dro.update_status(status_dict)

        # Update viewport
        self._view_3d.update_status(status_dict)

        # Update sliders
        self._control.update_joint_sliders(status_dict['jnt_coords_deg'])

    def on_viewport_tab_change(self, index):

        widget = self._gui.viewport_tabs.widget(index)

        if widget is self._view_camera:
            self._camera.connect()
        else:
            self._camera.disconnect()