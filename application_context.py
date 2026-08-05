""" Application context stores whole application state.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

from backend.robot_system import RobotSystem
from backend.camera_manager import CameraManager
from backend.vision_manager import VisionManager
from backend.gc_serial import GCSerial
from robot_program.program_controller import ProgramController
from qt_gui.viewport.scene import *


class ApplicationContext:

    def __init__(self):
        # Components
        self.serial = GCSerial()
        self.robot_sys = RobotSystem()
        self.camera = CameraManager()
        self.vision = VisionManager()
        self.scene = SceneGraph(root=SceneNode(name="Scene"))
        self.scene.build_from_directory('scene/')
        self.program_controller = ProgramController(robot_sys=self.robot_sys)


        # self.program_parser = parser.ProgramParser()
        # self.program = None
        # self.executor = InstructionExecutor(parent=self.robot_sys)

        self.connect_signals()

    def connect_signals(self):
        """ Signal connections between context components
        """
        # ========================================= Manipulator ==========================================
        self.robot_sys.g_code_generated.connect(self.serial.send)

        # =========================================== Camera =============================================
        self.camera.frame_received.connect(self.vision.process_frame)

        # =========================================== Serial =============================================
