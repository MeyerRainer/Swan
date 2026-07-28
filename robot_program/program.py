from typing import List, Dict

from robot_program.instructions.instruction import Instruction
from robot_program.target import Target


class Program:

    def __init__(self, name: str = ""):

        self.name = name
        self.instructions: List[Instruction] = []
        self.targets: Dict[str, Target] = {}
        self.variables = {}

    def append(self, instruction: Instruction):
        self.instructions.append(instruction)

    def insert(self, index: int, instruction: Instruction):
        self.instructions.insert(index, instruction)

    def remove(self, index: int):
        del self.instructions[index]

    def clear(self):
        self.instructions.clear()

    def __iter__(self):
        return iter(self.instructions)

    def __len__(self):
        return len(self.instructions)
