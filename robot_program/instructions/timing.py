from robot_program.instructions.instruction import Instruction

from dataclasses import dataclass


@dataclass
class WaitSeconds(Instruction):
    seconds: float
    line: int