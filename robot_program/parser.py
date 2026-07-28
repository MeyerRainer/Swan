from abc import ABC, abstractmethod
import ast
from robot_program.program import Program
from robot_program.target import Target
from robot_program.instructions.motion import *
from robot_program.instructions.flow import *
from robot_program.condition import *
from robot_program.instructions.timing import *
import numpy as np

ALLOWED_STATEMENTS = {
    ast.Assign,
    ast.Expr,
    ast.If,
    ast.While,
    ast.Break,
    ast.Continue,
    ast.Return,
    ast.FunctionDef,
}

ALLOWED_FUNCTIONS = {
    "MoveJ",
    "MoveL",
    "MoveC",
    "Wait",
    "SetDO",
    "DI",
    "JointTarget",
    "PoseTarget",
}


class ProgramParser:

    def __init__(self):

        self.program: Program | None = None

    def parse(self, file_name: str, extension: str) -> Program:
        file: str = file_name + extension

        with open(file) as f:
            tree: ast.Module = ast.parse(f.read())

        self.program = Program(name=file_name)

        # Recursively parse abstract syntax tree
        self.program.instructions = self.parse_block(tree.body)

        return self.program

    def parse_block(self, statements: list[ast.stmt]) -> list[Instruction]:

        instructions = []

        for statement in statements:
            instructions.append(self.parse_statement(statement))

        return instructions

    def parse_statement(self, stmt: ast.stmt) -> Instruction | None:

        # Invalid statement
        if type(stmt) not in ALLOWED_STATEMENTS:
            raise SyntaxError(f"{type(stmt).__name__} is not part of the Swan language.")

        # Assignment
        if isinstance(stmt, ast.Assign):
            return self.parse_assignment(stmt)

        # Expression
        if isinstance(stmt, ast.Expr):
            expr = stmt.value

            # Function call
            if isinstance(expr, ast.Call):
                if not isinstance(expr.func, ast.Name):
                    raise SyntaxError(f"Expected function name, line {expr.lineno}.")

                function_name = expr.func.id

                if function_name not in ALLOWED_FUNCTIONS:
                    raise SyntaxError(f"Unknown robot instruction '{function_name}', line {expr.lineno}.")

                return self.parse_call(expr, stmt.lineno)

        # If-statement
        elif isinstance(stmt, ast.If):
            condition = self.parse_condition(stmt.test)
            body = self.parse_block(stmt.body)
            return If(line=stmt.lineno, condition=condition, body=body)

        # While-loop
        elif isinstance(stmt, ast.While):
            condition = self.parse_condition(stmt.test)
            body = self.parse_block(stmt.body)
            return While(line=stmt.lineno, condition=condition, body=body)

        return None

    def parse_assignment(self, stmt: ast.Assign) -> None:

        if len(stmt.targets) != 1:
            raise SyntaxError(f"Only single assignments are supported, line {stmt.lineno}.")

        target = stmt.targets[0]

        if not isinstance(target, ast.Name):
            raise SyntaxError(f"Expected variable name, line {stmt.lineno}.")

        variable_name = target.id

        value = stmt.value

        if not isinstance(value, ast.Call):
            raise SyntaxError(f"Expected JointTarget(...), line {stmt.lineno}.")

        if not isinstance(value.func, ast.Name):
            raise SyntaxError(f"Expected constructor name, line {stmt.lineno}.")

        constructor = value.func.id

        if constructor == "JointTarget":
            robot_target = self.parse_joint_target(variable_name, value)

            self.program.targets[variable_name] = robot_target

            return None

        raise SyntaxError(f"Unknown constructor '{constructor}', line {stmt.lineno}.")

    def parse_call(self, call: ast.Call, line: int) -> Instruction:
        if not isinstance(call.func, ast.Name):
            raise SyntaxError("Expected function name.")

        name = call.func.id

        if name == "MoveJ":
            if len(call.args) != 1:
                raise SyntaxError("MoveJ expects one argument.")

            arg = call.args[0]
            if not isinstance(arg, ast.Name):
                raise SyntaxError("MoveJ expects a target.")

            return MoveJ(target=self.program.targets[arg.id], line=line)

        raise SyntaxError(f"Unknown instruction '{name}'.")

    def parse_condition(self, expr: ast.expr) -> Condition:

        if not isinstance(expr, ast.Call):
            raise SyntaxError("Expected function call.")

        if not isinstance(expr.func, ast.Name):
            raise SyntaxError("Expected function name.")

        if expr.func.id != "DI":
            raise SyntaxError("Unknown condition.")

        if len(expr.args) != 1:
            raise SyntaxError("DI expects one argument.")

        arg = expr.args[0]

        if not isinstance(arg, ast.Constant):
            raise SyntaxError("Signal name must be a string.")

        if not isinstance(arg.value, str):
            raise SyntaxError("Signal name must be a string.")

        return DigitalInputCondition(arg.value)

    def parse_joint_target(self, name: str, call: ast.Call) -> Target:

        values = []

        for arg in call.args:

            if not isinstance(arg, ast.Constant):
                raise SyntaxError("JointTarget only accepts numeric constants.")

            values.append(float(arg.value))

        return Target(name=name, joints=np.array(values))
