from abc import ABC
from dataclasses import dataclass


@dataclass
class Instruction(ABC):
    """ Base class for all robot instructions. """

    line: int  # Line number

    pass

# class Instruction:
#
#     def __init__(self, line: int):
#
#         self.line: int = line
#