from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QWidget

from robot_program.executor import InstructionExecutor
from robot_program.parser import ProgramParser
from robot_program.program import Program
from robot_program.writer import ProgramWriter


class ProgramManager(QWidget):

    write_terminal = pyqtSignal(str)

    def __init__(self, robot_sys):

        super().__init__()

        self.parser = ProgramParser()
        # self.writer = ProgramWriter()
        self.executor = InstructionExecutor()
        self.executor.set_robot_system(robot_sys)

        self.program: Program | None = None

    def load_program(self, file: str, extension: str):
        self.program = self.parser.parse(file, extension)
        self.write_terminal.emit(f"Program loaded: {self.program.name}")

    def get_program(self) -> Program:
        return self.program

    def execute_program(self):
        if self.program is None:
            self.write_terminal.emit("No program loaded.")
            return
        self.executor.execute(self.program)
