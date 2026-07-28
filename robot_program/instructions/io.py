""" IO-instructions """

from dataclasses import dataclass

from robot_program.instructions.instruction import Instruction


@dataclass
class DI(Instruction):
    ...

@dataclass
class DO(Instruction):
    ...
