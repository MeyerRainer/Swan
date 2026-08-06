from backend.robot_system import RobotSystem
from robot_program.program_context import *
from robot_program.program_executor import *
import robot_program.api as api

import traceback
from typing import Optional, Dict, Any
from PyQt6.QtCore import QObject, pyqtSignal


class ProgramManager(QObject):

    sgn_write_terminal = pyqtSignal(str)
    sgn_program_loaded = pyqtSignal(str, str)  # File name and contents.
    sgn_current_program_line = pyqtSignal(int)

    def __init__(self, robot_sys: RobotSystem):

        super().__init__()

        self.robot_driver: RobotSystem = robot_sys

        # Program executor running on another worker thread.
        self.executor: Optional[ProgramExecutor] = None
        self.program_file_path: Optional[str] = None

        # Internal program structure. (Tree)
        self.program: Optional[BlockNode] = None
        self.targets = Dict[str, Target]

    def load_program(self, file_name: str, file_extension: str) -> bool:
        self.program_file_path = file_name + file_extension
        context = ProgramContext()
        if self._execute(self.program_file_path, context):
            self.program = context.root
            self.targets = context.targets
            self.sgn_write_terminal.emit(f"Program loaded: {self.program_file_path}")
            self.sgn_program_loaded.emit(file_name, file_extension)
            print(f"Program loaded")
            return True
        self.sgn_write_terminal.emit(f"Loading failed.: {self.program_file_path}")
        print(f"Loading failed")
        return False

    # Run internal program.
    def on_run(self):
        # Paused by default.
        self.executor = ProgramExecutor(self.program, self.robot_driver)
        # Start executor thread.
        self.executor.start()

    def on_pause(self):
        if self.executor:
            self.executor.pause()

    def on_resume(self):
        if self.executor:
            self.executor.resume()

    def on_step(self):
        if self.executor:
            self.executor.step()

    def on_stop(self):
        if self.executor:
            self.executor.stop()

    @staticmethod
    def _execute(file_path: str, context: ProgramContext) -> bool:

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                source_code = f.read()

            # Compile to bytecode.
            code_obj = compile(source_code, filename=file_path, mode="exec")

            # Inject execution scope. (build, real, simulation)
            execution_scope = {
                "__file__":         file_path,
                "__name__":         "__main__",
                "__builtins__":     __builtins__,
            }

            api.set_thread_context(context)
            exec(code_obj, execution_scope)

            return True

        except SyntaxError as se:
            print(f"[ProgramManager] Syntax Error on line {se.lineno}: {se.msg}")
            return False

        except Exception as e:
            print(f"[ProgramManager] Runtime Error during execution:")
            traceback.print_exc(limit=3)
            return False

        finally:
            api.set_thread_context(None)