from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QWidget

from robot_program.executor import InstructionExecutor
from robot_program.parser import ProgramParser
from robot_program.program import Program
from robot_program.writer import ProgramWriter


class ProgramController(QWidget):

    write_terminal = pyqtSignal(str)
    sgn_program_loaded = pyqtSignal(str, str)  # File name and contents

    def __init__(self, robot_sys):

        super().__init__()

        self.parser = ProgramParser()
        # self.writer = ProgramWriter()
        self.executor = InstructionExecutor()
        self.executor.set_robot_system(robot_sys)

        self.program: Program | None = None

    @property
    def program_loaded(self):
        return self.program is not None

    # def load_program(self, file: str, extension: str):
    #     self.program = self.parser.parse(file, extension)
    #     self.write_terminal.emit(f"Program loaded: {self.program.name}")

    def load_program(self, file_name: str, extension: str):

        file: str = file_name + extension

        with open(file) as f:
            contents = f.read()

        self.program: Program = self.parser.parse(contents)

        self.sgn_program_loaded.emit(file, contents)  # Send to Program panel
        self.write_terminal.emit(f"Program loaded: {self.program.name}")

    def get_program(self) -> Program:
        return self.program

    def execute_program(self):
        if self.program is None:
            self.write_terminal.emit("No program loaded.")
            return
        self.executor.execute(self.program)
