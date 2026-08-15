from robot_program.api import *
import numpy as np
from robot_math.pose import Pose
import math

# home = Target(name="Home", joints=np.array([45., 0., 0., 0., 0., 0., 0., 0.]))
# pick = Target(name="Pick", joints=np.array([45., 20., 0., 0., 0., 0., 0., 0.]))
home = Target(name="Home", pose=Pose.from_zyz_euler(pos=np.array([0.260, 0.000, 0.320]), zyz=np.array([0., math.pi/2, math.pi])))
pick = Target(name="Pick", pose=Pose.from_zyz_euler(pos=np.array([0.260, 0.000, 0.280]), zyz=np.array([0., math.pi/2, math.pi])))

# home = Target(Pose.from_position(np.array([0.1, 0.1, 0.1])))
# pick = Target(Pose.from_position(np.array([0.1, 0.1, 0.2])))

MoveCartesianLinear(target=home, speed=1000.)
MoveCartesianLinear(target=pick, speed=1000.)
MoveCartesianLinear(target=home, speed=1000.)
MoveCartesianLinear(target=pick, speed=1000.)
MoveCartesianLinear(target=home, speed=1000.)



# print(f"Hello test program")

# if not DigitalIn(1):
#     Wait(0.5)
#     MoveJ(pick)
