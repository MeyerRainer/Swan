from PyQt6.QtCore import pyqtSignal, QObject
from robot_program.program import Program
from robot_program.instructions.motion import *


class InstructionExecutor(QObject):

    current_lineno = pyqtSignal(int)

    def __init__(self):

        super().__init__()

        self.robot_sys = None

    def set_robot_system(self, sys):
        self.robot_sys = sys

    def execute(self, program: Program):
        pass

        for instruction in program:

            self.current_lineno.emit(instruction.line)

            if isinstance(instruction, MoveJ):

                self.execute_move_joint(instruction)

            elif isinstance(instruction, MovePoseL):

                self.execute_move_linear(instruction)

    def execute_move_joint(self, instruction: MoveJ):
        jnt_vec = instruction.target.joints
        speed = instruction.target.speed
        print(type(jnt_vec))
        self.robot_sys.move_jnt(jnt_vec, speed)

    def execute_move_linear(self, instruction: MovePoseL):
        ...