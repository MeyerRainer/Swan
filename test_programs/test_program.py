from robot_program.api import *
import numpy as np

home = Target(name="Home", joints=np.array([45., 0., 0., 0., 0., 0., 0., 0.]))
pick = Target(name="Pick", joints=np.array([45., 20., 0., 0., 0., 0., 0., 0.]))
# home = Target(Pose.from_position(np.array([0.1, 0.1, 0.1])))
# pick = Target(Pose.from_position(np.array([0.1, 0.1, 0.2])))

MoveJ(home)
MoveJ(pick)
MoveJ(home)
MoveJ(pick)
MoveJ(home)
MoveJ(pick)
MoveJ(home)
MoveJ(pick)
MoveJ(home)
MoveJ(pick)
MoveJ(home)
MoveJ(pick)
MoveJ(home)
MoveJ(pick)

# if not DigitalIn(1):
#     Wait(0.5)
#     MoveJ(pick)
