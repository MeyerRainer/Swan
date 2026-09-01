""" Internal data structure of a robot program.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from typing import Dict
from robot_program.instructions.instruction import *


class ProgramContext:

    def __init__(self):

        self.root: ProgramRoot = ProgramRoot()      # Program tree
        self.targets: Dict[str, Target] = {}    # Targets

    def move_j(self, target: Target, speed: float, lineno: int) -> None:
        self.targets[target.name] = target
        self.root.children.append(MoveJointNode(target=target, speed=speed, lineno=lineno))
        # print(f"Child node appended. Current length of children: {len(self.root.children)}")

    def move_cartesian_joint(self, target: Target, speed: float, lineno: int) -> None:
        self.targets[target.name] = target
        self.root.children.append(MoveCartesianJointNode(target=target, speed=speed, lineno=lineno))

    def move_cartesian_linear(self, target: Target, speed: float, lineno: int) -> None:
        self.targets[target.name] = target
        self.root.children.append(MoveCartesianLinearNode(target=target, speed=speed, lineno=lineno))

    def move_cartesian_circle(self, target: Target, center: Target, speed: float, lineno: int) -> None:
        self.targets[target.name] = target
        self.root.children.append(MoveCartesianLinearNode(target=target, speed=speed, lineno=lineno))

    def wait(self, seconds: float, line_no: int):
        self.root.children.append(WaitSecondsNode(seconds=seconds, line_no=line_no))

    def digital_in(self, pin: int) -> bool:
        # Build-time condition evaluation or placeholder logic
        return False