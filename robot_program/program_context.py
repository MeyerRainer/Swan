from robot_program.node import *
from robot_program.instructions.instruction import *

import sys
import traceback
from dataclasses import dataclass, field
from typing import List, Optional, Any, Dict
from abc import ABC, abstractmethod


class ProgramContext:

    def __init__(self):

        self.root: BlockNode = BlockNode()      # Program tree
        self.targets: Dict[str, Target] = {}    # Targets

    def move_j(self, target: Target, speed: float, lineno: int):
        self.targets[target.name] = target
        self.root.children.append(MoveJNode(target=target, speed=speed, lineno=lineno))
        # print(f"Child node appended. Current length of children: {len(self.root.children)}")

    def wait(self, seconds: float, line_no: int):
        self.root.children.append(WaitNode(seconds=seconds, line_no=line_no))

    def digital_in(self, pin: int) -> bool:
        # Build-time condition evaluation or placeholder logic
        return False