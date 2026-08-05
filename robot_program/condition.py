from abc import ABC
from dataclasses import dataclass, field
from typing import Any, List

from robot_program.instructions.instruction import Instruction


class Condition(ABC):
    """ Base class for all boolean conditions.

    Conditions are parsed from the source program and later
    evaluated by the robot executor.
    """
    pass

@dataclass
class DigitalInputCondition(Condition):

    def __init__(self, signal: str):
        self.signal = signal

    signal: str


@dataclass
class IfCondition(Instruction):
    condition: Any
    body: List[Instruction] = field(default_factory=list)
    line: int
