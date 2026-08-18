""" Application current_context stores whole application state.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtCore import QThread

from qt_gui.viewport.scene import *

from backend.gc_serial import GCSerial
from backend.robot_system import RobotSystem
from backend.camera_manager import CameraManager
from vision.camera import CameraWorker
from vision.vision_manager import VisionManager
from robot_program.program_manager import ProgramManager
from planner.trajectory_planner import TrajectoryPlanner


class ApplicationContext:

    def __init__(self):

        # Components
        self.serial = GCSerial()
        self.robot_sys = RobotSystem()
        # self.camera = CameraManager()
        self.camera: CameraWorker = CameraWorker()
        self.vision_manager = VisionManager()
        self.scene = SceneGraph(root=None, dir_path="scene/")
        self.planner = TrajectoryPlanner()
        self.program_manager = ProgramManager(planner=self.planner)

        # print(f"App context: Number of scene nodes: {self.scene.size()}")

        self.connect_signals()

    def connect_signals(self):
        """ Connections between application context components
        """
        # ========================================= Manipulator ==========================================
        self.robot_sys.g_code_generated.connect(self.serial.send)

        # =========================================== Camera =============================================
        self.camera.sgn_frame_received.connect(self.vision_manager.process_frame)

        # =========================================== Serial =============================================

    def update_status(self, status, m_pos, delta_t) -> dict:
        status_dict: dict = self.robot_sys.update_status(status, m_pos, delta_t)
        self.scene.update()

        return status_dict

    def on_shutdown(self):
        self.camera.shutdown()
