""" Application current_context stores whole application state.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from application_settings import ApplicationSettings
from robot_manipulator.gc_serial import GCSerial
from robot_manipulator.robot_system import RobotSystem
from qt_gui.viewport.scene.scene import SceneGraph
from vision.vision_system import VisionSystem
from robot_program.program_manager import ProgramManager
from planner.trajectory_planner import TrajectoryPlanner


class ApplicationContext:

    def __init__(self):

        # Components
        self.settings = ApplicationSettings()
        self.serial = GCSerial()
        self.robot_sys = RobotSystem()
        self.vision_sys = VisionSystem()
        self.scene = SceneGraph(robot_sys=self.robot_sys, root=None, dir_path="scene/")
        self.planner = TrajectoryPlanner()
        self.program_manager = ProgramManager(planner=self.planner)
        # self.scene.print_tree()

        self.connect_signals()
        self.init_application_context()

    def init_application_context(self):
        self.vision_sys.start()

    def connect_signals(self):
        """ Connections between application context components
        """
        # ========================================= Manipulator ==========================================
        self.robot_sys.g_code_generated.connect(self.serial.send)

        # =========================================== Camera =============================================
        # self.camera.sgn_frame_received.connect(self.vision_manager.process_frame)

        # =========================================== Serial =============================================

    def update_status(self, status, m_pos, delta_t) -> dict:
        status_dict: dict = self.robot_sys.update_status(status, m_pos, delta_t)

        return status_dict
