from backend.robot_system import RobotSystem
from backend.camera_manager import CameraManager
from backend.vision_manager import VisionManager
from backend.gc_serial import GCSerial
from robot_program.program_manager import ProgramManager
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
        # self.program = ProgramManager()


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

        # ports_changed = pyqtSignal(list)
        # line_received = pyqtSignal(str)
        # status_received = pyqtSignal(str)
        # error_received = pyqtSignal(str)
        # alarm_received = pyqtSignal(str)

        # self.serial.ports_changed.connect() # ?
        # self.serial.line_received.connect(self.)  # Serial process line?
        # self.serial.error_received.connect()  # ?
        # self.serial.alarm_received.connect() # ?
