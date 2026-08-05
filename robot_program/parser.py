from robot_program.instructions.timing import WaitSeconds
from robot_program.program import Program
from robot_program.target import Target
from robot_program.instructions.motion import *
from robot_program.instructions.flow import *
from robot_program.condition import *

from typing import Any, Optional
import ast
import numpy as np

# ALLOWED_STATEMENTS = {
#     ast.Assign,
#     ast.Expr,
#     ast.If,
#     ast.For,
#     ast.While,
#     ast.Break,
#     ast.Continue,
#     ast.Return,
#     ast.FunctionDef,
#     ast.Call,
# }
#
# # Custom functions for controlling robot system
# ALLOWED_FUNCTIONS = {
#     "MoveJ",
#     "MovePoseJ",
#     "MovePoseX",
#     "Wait",
#     "SetDO",
#     "DI",
#     "JointTarget",
#     "PoseTarget",
# }


class ProgramParser(ast.NodeVisitor):

    def __init__(self):

        self.program = Program()

    def parse(self, code_str: str) -> Program:
        """ Parses Python source code into an internal Program representation.
        """

        ast_root: ast.Module = ast.parse(code_str)
        self.visit(ast_root)
        return self.program

    def visit_Assign(self, node: ast.Assign):
        """Handles target definitions and variable assignments:
        home = TargetJ(0, 0, 0, 0, 0, 0)
        """
        target_name = node.targets[0].id  # Left side variable name
        value_node = node.value

        # Check if right-hand side is a Target creation
        if isinstance(value_node, ast.Call) and isinstance(value_node.func, ast.Name):
            func_name = value_node.func.id

            if func_name == "TargetJ":
                # Evaluate static numerical arguments for joint angles
                args = [ast.literal_eval(arg) for arg in value_node.args]
                target_obj = Target(name=target_name, joints=np.array(args))
                self.program.targets[target_name] = target_obj
                return

        # Regular variable assignments fallback
        if isinstance(value_node, ast.Constant):
            self.program.variables[target_name] = value_node.value

    def visit_Expr(self, node: ast.Expr):
        """Handles standalone commands like MoveJ(home) or Wait(1.5)."""
        if isinstance(node.value, ast.Call):
            instr = self._parse_instruction_call(node.value, node.lineno)
            if instr:
                self.program.instructions.append(instr)

    # TODO
        if isinstance(node.value, ast.Assign):
            instr = self._parse_assignment_call(node.value, node.lineno)
            if instr:
                self.program.targets['kk'] = instr

    def visit_If(self, node: ast.If):
        """Handles control flow blocks: if DigitalIn('sensor1'): ..."""
        condition = self._parse_condition(node.test)

        # Save outer scope instruction list and parse inner block
        outer_instructions = self.program.instructions
        self.program.instructions = []

        for body_stmt in node.body:
            self.visit(body_stmt)

        inner_instructions = self.program.instructions
        self.program.instructions = outer_instructions

        self.program.instructions.append(IfCondition(condition=condition, body=inner_instructions, line=node.lineno))

    def _parse_instruction_call(self, call_node: ast.Call, lineno: int) -> Optional[Instruction]:
        """Translates ast.Call nodes to custom robot Instructions."""
        func_name = call_node.func.id

        if func_name == "MoveJ":
            target_arg = call_node.args[0]
            if isinstance(target_arg, ast.Name):
                target_name = target_arg.id
                target = self.program.targets.get(target_name, Target(name=target_name))
                return MoveJ(target=target, line=lineno)

        elif func_name == "Wait":
            seconds = ast.literal_eval(call_node.args[0])
            return WaitSeconds(seconds=seconds, line=lineno)

        return None

    def _parse_condition(self, test_node: ast.AST) -> Any:
        """Parses condition expressions into Condition objects."""
        # Handles DigitalIn("signal_name")
        if isinstance(test_node, ast.Call) and isinstance(test_node.func, ast.Name):
            if test_node.func.id == "DigitalIn":
                signal = ast.literal_eval(test_node.args[0])
                return DigitalInputCondition(signal=signal)

        # Handles unary operations like: `not DigitalIn(...)`
        if isinstance(test_node, ast.UnaryOp) and isinstance(test_node.op, ast.Not):
            inner_cond = self._parse_condition(test_node.operand)
            return "NOT", inner_cond

        return None

    def _parse_assignment(self, stmt: ast.Assign) -> None:

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