from abc import ABC, abstractmethod
import ast
from program.program import Program
from instructions.motion import *
from instructions.timing import *

# class ProgramParser(ABC):
#
#     @abstractmethod
#     def parse(self, filename: str) -> Program:
#         """Read file and return RobotProgram."""
#         raise NotImplementedError


class ProgramParser:

    def parse(self, filename: str) -> Program:
        with open(filename) as f:
            tree = ast.parse(f.read())

        program = Program()

        for statement in tree.body:
            instruction = self.parse_statement(statement)
            program.append(instruction)

        return program

    def parse_statement(self, node):
        if not isinstance(node, ast.Expr):
            raise SyntaxError()

        call = node.value
        name = call.func.id

        if name == "MoveJ":
            target_name = call.args[0].id
            return MoveJ(target_name)

        elif name == "MoveL":
            target_name = call.args[0].id
            return MoveL(target_name)

        # elif name == "Wait":
        #     return Wait(call.args[0].value)

        else:
            raise SyntaxError(f"Unknown command {name}")