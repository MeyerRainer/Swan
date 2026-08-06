from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Any

from robot_program.api import Target


class InstructionNode(ABC):
    """Base class for instructions in the internal tree."""
    def __init__(self, line_no: int = -1):
        self.line_no = line_no

    @abstractmethod
    def execute(self, driver: Any) -> None:
        """Executes the instruction directly on the physical hardware driver."""
        pass

@dataclass
class MoveJNode(InstructionNode):
    target: Target
    lineno: int = -1

    def __init__(self, target: Target, speed: float, lineno: int):

        super().__init__()

        self._target = target
        self._speed = speed
        self._lineno = lineno
        print(f"MoveJNode initialized")

    def execute(self, driver: Any) -> None:
        print(f"[Driver] Executing MoveJ to {self._target.name} (Line {self.line_no})")
        # Blocking motion command down to physical robot hardware/controller
        driver.move_jnt(jnt_vec=self._target.joints, speed=self._speed, degrees=True)

@dataclass
class WaitNode(InstructionNode):
    seconds: float
    line_no: int = -1

    def execute(self, driver: Any) -> None:
        print(f"[Driver] Executing Wait({self.seconds}s)")
        driver.sleep(self.seconds)


# Root
class BlockNode(InstructionNode):
    def __init__(self):
        super().__init__()
        self.children: List[InstructionNode] = []

    def execute(self, driver: Any) -> None:
        for child in self.children:
            child.execute(driver)