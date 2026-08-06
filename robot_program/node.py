# import sys
# import traceback
# from dataclasses import dataclass, field
# from typing import List, Optional, Any, Dict
# from abc import ABC, abstractmethod
#
#
# # Datastructures for internal representation of program.
# class ProgramNode(ABC):
#     pass
#
# @dataclass
# class MotionNode(ProgramNode):
#     command: str  # "MoveJ", "MoveL"
#     target_name: str
#
# @dataclass
# class WaitNode(ProgramNode):
#     seconds: float
#
# @dataclass
# class BlockNode(ProgramNode):
#     children: List[ProgramNode] = field(default_factory=list)
#
# @dataclass
# class IfNode(ProgramNode):
#     condition_str: str
#     true_block: BlockNode
#     false_block: Optional[BlockNode] = None
