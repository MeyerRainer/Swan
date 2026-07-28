""" Motion instructions """

from dataclasses import dataclass

from robot_program.instructions.instruction import Instruction
from robot_program.target import Target


@dataclass
class MoveJ(Instruction):
    target: Target


@dataclass
class MoveL(Instruction):
    target: Target
