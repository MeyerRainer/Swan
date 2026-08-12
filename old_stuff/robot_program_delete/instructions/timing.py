from robot_program_delete.instructions.instruction import Instruction

from dataclasses import dataclass


@dataclass
class WaitSeconds(Instruction):
    seconds: float
    line: int