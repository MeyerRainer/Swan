from robot_math.pose import Pose
from dataclasses import dataclass
from typing import List, Dict, Optional, Any
import numpy as np
import threading
import inspect

# --- Thread-Local Storage for Context Isolation ---
_thread_local = threading.local()

def set_thread_context(context: Any) -> None:
    """Sets the active RobotContext for the current thread."""
    _thread_local.active_context = context

def get_thread_context() -> Any:
    """Retrieves the active RobotContext bound to the current thread."""
    ctx = getattr(_thread_local, "active_context", None)
    if ctx is None:
        raise RuntimeError(
            "Robot API function called outside of an active ProgramManager execution context."
        )
    return ctx


# --- Internal Helper for Line Tracking ---
def get_caller_line_no() -> int:
    """Retrieves the line number from the user's script that called the API function."""
    frame = inspect.currentframe()
    if frame and frame.f_back and frame.f_back.f_back:
        return frame.f_back.f_back.f_lineno
    elif frame and frame.f_back:
        return frame.f_back.f_lineno
    return -1


@dataclass
class Target:
    """ Robot target.

    Pose and joint values may both exist.
    Depending on the instruction only one may be used.
    """

    name: str = ""
    pose: Optional[Pose] = None      # Operational space
    joints: Optional[np.ndarray] = None  # Joint space

    tool: str = "tool0"
    frame: str = "world"
    speed: float = 100.0
    zone: float = 0.0

def MoveJ(target: Target, speed: float) -> None:
    lineno = get_caller_line_no()
    get_thread_context().move_j(target=target, speed=speed, lineno=lineno)

def Wait(seconds: float) -> None:
    pass

def DigitalIn(pin: int) -> bool:
    pass