from backend.robot_system import RobotSystem
from robot_program.program import Program
from robot_program.target import Target
from robot_program.instructions.motion import *
from robot_program.instructions.flow import *
from robot_program.condition import *
from robot_program.instructions.timing import *
import numpy as np


class InstructionExecutor:

    def __init__(self):

        self.robot_sys = None

    def set_robot_system(self, sys):
        self.robot_sys = sys

    def execute(self, program: Program):
        pass

        for instruction in program:

            if isinstance(instruction, MoveJ):

                self.execute_move_joint(instruction)

            elif isinstance(instruction, MoveL):

                self.execute_move_linear(instruction)

    def execute_move_joint(self, instruction: MoveJ):
        jnt_vec = instruction.target.joints
        speed = instruction.target.speed
        self.robot_sys.move_jnt(jnt_vec, speed)

    def execute_move_linear(self, instruction: MoveL):
        ...