""" Application current_context stores whole application state.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from qt_gui.viewport.scene import *

from backend.gc_serial import GCSerial
from backend.robot_system import RobotSystem
from backend.camera_manager import CameraManager
from backend.vision_manager import VisionManager
from robot_program.program_manager import ProgramManager
from planner.trajectory_planner import TrajectoryPlanner


class ApplicationContext:

    def __init__(self):
        # Components
        self.serial = GCSerial()
        self.robot_sys = RobotSystem()
        self.camera = CameraManager()
        self.vision = VisionManager()
        self.scene = SceneGraph(root=None, dir_path="scene/")
        self.program_manager = ProgramManager(robot_sys=self.robot_sys)
        self.planner = TrajectoryPlanner()

        # print(f"App context: Number of scene nodes: {self.scene.size()}")

        self.connect_signals()

    def connect_signals(self):
        """ Connections between application context components
        """
        # ========================================= Manipulator ==========================================
        self.robot_sys.g_code_generated.connect(self.serial.send)

        # =========================================== Camera =============================================
        self.camera.frame_received.connect(self.vision.process_frame)

        # =========================================== Serial =============================================

    def update_status(self, status, m_pos, delta_t) -> dict:
        return self.robot_sys.update_status(status, m_pos, delta_t)
