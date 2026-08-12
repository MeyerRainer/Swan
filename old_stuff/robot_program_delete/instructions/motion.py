""" Motion instructions """

from dataclasses import dataclass

from robot_program_delete.instructions.instruction import Instruction
from robot_program_delete.target import Target


@dataclass
class MoveJ(Instruction):
    target: Target


@dataclass
class MovePoseJ(Instruction):
    target: Target


@dataclass
class MovePoseL(Instruction):
    target: Target