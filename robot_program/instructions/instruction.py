from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Any

from robot_program.api import Target


class InstructionNode(ABC):
    """Base class for instructions in the internal tree."""
    def __init__(self, line_no: int = -1):
        self.line_no = line_no

    @abstractmethod
    def plan(self, planner: Any) -> None:
        pass


@dataclass
class MoveJointNode(InstructionNode):
    target: Target
    lineno: int = -1
    def __init__(self, target: Target, speed: float, lineno: int):
        super().__init__()
        self.target = target
        self.speed = speed
        self.lineno = lineno
        print(f"MoveJoint initialized")

    def plan(self, planner: Any) -> None:
        planner.process_instruction(self)


@dataclass
class MoveCartesianLinearNode(InstructionNode):
    target: Target
    lineno: int = -1
    def __init__(self, target: Target, speed: float, lineno: int):
        super().__init__()
        self.target = target
        self.speed = speed
        self.lineno = lineno
        print(f"MoveCartesianLinear initialized")

    def plan(self, planner):
        planner.process_instruction(self)


@dataclass
class MoveCartesianJointNode(InstructionNode):
    target: Target
    lineno: int = -1
    def __init__(self, target: Target, speed: float, lineno: int):
        super().__init__()
        self.target = target
        self.speed = speed
        self.lineno = lineno
        print(f"MoveCartesianJoint initialized")

    def plan(self, planner):
        planner.process_instruction(self)


@dataclass
class WaitSecondsNode(InstructionNode):
    seconds: float
    line_no: int = -1

    def execute(self, driver: Any) -> None:
        print(f"[Driver] Executing Wait({self.seconds}s)")
        driver.sleep(self.seconds)

    def plan(self, planner: Any):
        planner.process_instruction(self)


# Root
class ProgramRoot(InstructionNode):
    def __init__(self):
        super().__init__()
        self.children: List[InstructionNode] = []

    def plan(self, planner: Any) -> None:
        for child in self.children:
            child.plan(planner)
