from backend.robot_system import RobotSystem
from robot_program.program_context import *
from robot_program.api import Target
import robot_program.api as api

import traceback
from typing import Optional, Dict, Any
from PyQt6.QtCore import QObject, pyqtSignal, QThread
from enum import Enum, auto
import threading


class ExecutionState(Enum):
    RUNNING = auto()
    PAUSED = auto()
    STEPPING = auto()
    STOPPED = auto()


class ExecutionController:

    def __init__(self):

        self.state = ExecutionState.RUNNING
        self._step_event = threading.Event()
        self._step_event.set()  # Set means "keep running"
        self._stop_requested = False
        self.current_line = -1

    def pause(self):
        print(f"Pause")
        self.state = ExecutionState.PAUSED
        self._step_event.clear()  # Blocks the trace hook

    def resume(self):
        print(f"Resume")
        self.state = ExecutionState.RUNNING
        self._step_event.set()

    def step(self):
        print(f"Step")
        """Advances execution by exactly one line, then pauses again."""
        self.state = ExecutionState.STEPPING
        self._step_event.set()

    def stop(self):
        print(f"Stop")
        self.state = ExecutionState.STOPPED
        self._stop_requested = True
        self._step_event.set()  # Unblock thread so it can exit


class ProgramManager(QObject):

    sgn_write_terminal = pyqtSignal(str)
    sgn_program_loaded = pyqtSignal(str, str)  # File name and contents

    def __init__(self, robot_sys: RobotSystem):

        super().__init__()

        self.current_context: Optional[RobotContext] = None
        self.program: Optional[BlockNode] = None
        self.targets: Dict[str, Target]

        self.program_file_path: Optional[str] = None
        self.robot_driver: RobotSystem = robot_sys

        self.worker_thread = None
        self.controller = ExecutionController()
        self.line_changed_callback = None

    def load_program(self, file_name: str, file_extension: str) -> bool:
        context = ProgramBuilderContext()
        self.program_file_path = file_name + file_extension
        if self._execute(self.program_file_path, context):
            self.sgn_write_terminal.emit(f"Program loaded: {self.program_file_path}")
            return True
        return False

    # Execute user program to build internal datastructures.
    def execute_program(self):
        # if self.program_file_path is None:
        #     self.sgn_write_terminal.emit("Program not loaded.")
        #     return
        # if self.run_robot_sys(self.program_file_path, self.robot_driver):
        #     self.sgn_write_terminal.emit("Program executed.")
        if self.program_file_path is None:
            self.sgn_write_terminal.emit("Program not loaded.")
            return

        context = RealRobotContext(self.robot_driver)

        # Create worker thread so GUI thread never blocks!
        self.worker_thread = ScriptExecutionThread(self, self.program_file_path, context)
        self.worker_thread.finished_signal.connect(self._on_execution_finished)
        self.worker_thread.start()

    def _on_execution_finished(self, success: bool):
        if success:
            self.sgn_write_terminal.emit("Program executed successfully.")
        else:
            self.sgn_write_terminal.emit("Program execution stopped or failed.")

    # Execute user program to run real hardware.
    def run_robot_sys(self, file_path, hardware_driver: RobotSystem) -> bool:
        context: RobotContext = RealRobotContext(hardware_driver)
        return self._execute(file_path, context)

    def _execute(self, file_path: str, context: RobotContext) -> bool:
        self.current_context = context  # TODO: Delete
        api.set_thread_context(self.current_context)

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                source_code = f.read()

            # Compile to bytecode.
            code_obj = compile(source_code, filename=file_path, mode="exec")

            # Inject execution scope. (build, real, simulation)
            execution_scope = self._create_execution_scope(file_path)

            # Execute the bytecode in the isolated scope.
            print(f"Exec")
            # Set trace callback for this thread ONLY
            sys.settrace(self._trace_hook)
            exec(code_obj, execution_scope)

            return True

        except ScriptStoppedException:
            print("[ProgramManager] Execution stopped cleanly by user.")
            return False

        except SyntaxError as se:
            print(f"[ProgramManager] Syntax Error on line {se.lineno}: {se.msg}")
            return False

        except Exception as e:
            print(f"[ProgramManager] Runtime Error during execution:")
            traceback.print_exc(limit=3)
            return False

        finally:
            sys.settrace(None)
            api.set_thread_context(None)
            self.current_context = None

    @staticmethod
    def _create_execution_scope(file_path: str) -> Dict[str, Any]:
        """Creates a custom dictionary containing injected API functions bound to current_context."""

        # Bind API calls directly to the current_context methods passed into this instance
        return {
            "__file__": file_path,
            "__name__": "__main__",
            "__builtins__": __builtins__,  # Provide standard built-ins (len, range, print, etc.)
        }

    def _trace_hook(self, frame, event, arg):
        """Fires before every line of Python code is executed."""
        if event == "line":
            # Filter trace events to ONLY track lines from the user's script
            if frame.f_code.co_filename != self.program_file_path:
                return self._trace_hook

            line_no = frame.f_lineno
            self.controller.current_line = line_no

            # Notify GUI of active execution line for UI highlighting
            if self.line_changed_callback:
                self.line_changed_callback(line_no)

            # Check if user requested an immediate abort
            if self.controller._stop_requested:
                raise ScriptStoppedException("Program stopped by user")

            # Block thread if paused
            self.controller._step_event.wait()

            # If we were in single-step mode, re-pause immediately for the next step
            if self.controller.state == ExecutionState.STEPPING:
                self.controller.state = ExecutionState.PAUSED
                self.controller._step_event.clear()

        return self._trace_hook

class ScriptStoppedException(Exception):
    """Custom exception to break out of exec() stack gracefully."""
    pass


class ScriptExecutionThread(QThread):
    finished_signal = pyqtSignal(bool)

    def __init__(self, program_manager: "ProgramManager", file_path: str, context: RobotContext):
        super().__init__()
        self.pm = program_manager
        self.file_path = file_path
        self.context = context

    def run(self):
        # Executes exec() on a background thread!
        success = self.pm._execute(self.file_path, self.context)
        self.finished_signal.emit(success)