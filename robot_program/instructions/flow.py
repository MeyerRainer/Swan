""" Program flow instructions """

from robot_program.instructions.instruction import Instruction
from robot_program.condition import *

from dataclasses import dataclass, field


@dataclass
class If(Instruction):

    condition: Condition

    body: list[Instruction] = field(default_factory=list)

@dataclass
class While(Instruction):
    condition: Condition

    body: list[Instruction] = field(default_factory=list)

@dataclass
class For(Instruction):
    ...