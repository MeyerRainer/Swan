from dataclasses import dataclass

from program.instructions.instruction import Instruction
from ..target import Target


@dataclass
class MoveJ(Instruction):
    target: Target


@dataclass
class MoveL(Instruction):
    target: Target
