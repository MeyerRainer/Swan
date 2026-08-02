from backend.robot_system import RobotSystem
from backend.camera_manager import CameraManager
from backend.vision_manager import VisionManager
from backend.gc_serial import GCSerial

import utils
from qt_gui.viewport.scene import *

from robot_program import parser
from robot_program.executor import InstructionExecutor

from PyQt6.QtCore import QObject
import numpy as np
import serial

from robot_program.program import Program

class ApplicationContext:

    def __init__(self):
        # Components
        self.serial = GCSerial()
        self.robot_sys = RobotSystem()
        self.camera = CameraManager()
        self.vision = VisionManager()
        self.scene = SceneTreeModel(root=SceneNode(name="Scene"))
        self.program_parser = parser.ProgramParser()
        self.program = None
        self.executor = InstructionExecutor(parent=self.robot_sys)

        self.connect_signals()

    def connect_signals(self):
        """ Signal connections between context components
        """
        # ========================================= Manipulator ==========================================
        self.robot_sys.g_code_generated.connect(self.serial.send)

        # =========================================== Camera =============================================
        self.camera.frame_received.connect(self.vision.process_frame)

        # =========================================== Serial =============================================

        # ports_changed = pyqtSignal(list)
        # line_received = pyqtSignal(str)
        # status_received = pyqtSignal(str)
        # error_received = pyqtSignal(str)
        # alarm_received = pyqtSignal(str)

        # self.serial.ports_changed.connect() # ?
        # self.serial.line_received.connect(self.)  # Serial process line?
        # self.serial.error_received.connect()  # ?
        # self.serial.alarm_received.connect() # ?

    # def load_program(self, file: str, extension: str):
    #     self.program = self.program_parser.parse(file, extension)
    #     self.terminal_panel.write(f"Program loaded: {self.program.name}")
    #
    # def get_program(self) -> Program:
    #     return self.program
    #
    # def execute_program(self):
    #     if self.program is None:
    #         self.terminal_panel.write("No program loaded.")
    #         return
    #     self.executor.execute(self.program)
