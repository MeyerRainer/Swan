from robot_math.pose import Pose
from robot_program.program_context import *

from dataclasses import dataclass
from typing import List, Dict, Optional, Any
import numpy as np
import threading
import inspect


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

# Thread-local storage ensures context isolation per execution thread
_thread_local = threading.local()

def set_thread_context(context: Any) -> None:
    _thread_local.active_context = context

def get_thread_context() -> Any:
    ctx = getattr(_thread_local, "active_context", None)
    if ctx is None:
        raise RuntimeError("Robot API function called outside of an active ProgramManager execution context.")
    return ctx


def MoveJ(target: Any) -> None:
    # Gets line number from caller frame
    frame = inspect.currentframe().f_back
    line_no = frame.f_lineno if frame else -1
    get_thread_context().move_j(target, line=line_no)

def Wait(seconds: float) -> None:
    get_thread_context().wait(seconds)

def DigitalIn(pin: int) -> bool:
    return get_thread_context().digital_in(pin)
