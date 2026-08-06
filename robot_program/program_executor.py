from robot_program.instructions.instruction import *

from PyQt6.QtCore import QThread, pyqtSignal
from enum import Enum, auto


class ExecutionState(Enum):
    STOPPED = auto()
    RUNNING = auto()
    PAUSED = auto()


class ProgramExecutor(QThread):

    sgn_instruction_changed = pyqtSignal(int)  # Emits current line_no to GUI
    sgn_finished = pyqtSignal()

    def __init__(self, program_root: BlockNode, driver: Any):

        super().__init__()

        self.program_node = program_root
        self.driver = driver

        self.current_index = 0
        self.state = ExecutionState.PAUSED
        self._step_requested = False
        self._stop_requested = False

    def resume(self):
        print("Resume")
        self.state = ExecutionState.RUNNING

    def pause(self):
        print("Pause")
        self.state = ExecutionState.PAUSED

    def step(self):
        print("Step")
        """Allows execution of exactly ONE instruction node."""
        self._step_requested = True

    def stop(self):
        print("Stop")
        self.state = ExecutionState.STOPPED
        self._stop_requested = True

    def run(self):
        print(f"Executor run.")
        instructions = self.program_node.children

        while self.current_index < len(instructions) and not self._stop_requested:
            print(f"While index: {self.current_index}")
            # Handle Pause / Step state waiting
            if self.state == ExecutionState.PAUSED:
                if self._step_requested:
                    self._step_requested = False
                    # Fall through to execute single instruction below
                else:
                    self.msleep(50)  # Yield CPU safely
                    print("Sleep")
                    continue

            # Get current instruction node
            print(f"Fetching node")
            node = instructions[self.current_index]

            # Signal GUI to highlight line
            self.sgn_instruction_changed.emit(node.line_no)

            # --- EXECUTE THE ACTUAL ROBOT INSTRUCTION ---
            print(f"Executing node")
            node.execute(self.driver)

            # Increment index
            self.current_index += 1

            # If we were stepping, revert to paused state after completing 1 instruction
            if self.state != ExecutionState.RUNNING:
                print(f"Pausing")
                self.state = ExecutionState.PAUSED

        print(f"Program Executor: Program finished")