from robot_program.node import *

import sys
import traceback
from dataclasses import dataclass, field
from typing import List, Optional, Any, Dict
from abc import ABC, abstractmethod


class RobotContext(ABC):

    @abstractmethod
    def move_j(self, target: Any, line: int): pass

    @abstractmethod
    def wait(self, seconds: float): pass

    @abstractmethod
    def digital_in(self, pin: int) -> bool: pass


# Program running context for constructing internal program datastructures.
class ProgramBuilderContext(RobotContext):

    def __init__(self):

        self.root = BlockNode()
        self.targets: Dict[str, Any] = {}
        print(f"ProgramBuilderContext initialized.")

    def move_j(self, target: Any, line: int):
        print(f"ProgramBuilderContext: move_j called.")
        self.targets[target.name] = target
        self.root.children.append(MotionNode(command="MoveJ", target_name=target.name))

    def wait(self, seconds: float):
        self.root.children.append(WaitNode(seconds=seconds))

    def digital_in(self, pin: int) -> bool:
        return False


# Program running context for real hardware.
class RealRobotContext(RobotContext):

    def __init__(self, robot_driver):

        self.driver = robot_driver
        print(f"RealRobotContext created with  driver: {robot_driver}")

    def move_j(self, target: Any, line: int):
        # Sends trajectory commands down to physical controller
        print(f"RealRobotContext move_j called.")
        self.driver.move_jnt(jnt_vec=target.joints, speed=target.speed, degrees=True)

    def wait(self, seconds: float):
        self.driver.sleep(seconds)

    def digital_in(self, pin: int) -> bool:
        return self.driver.get_gpio(pin)
