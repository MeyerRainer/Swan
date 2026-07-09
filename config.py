"""
Configuration file for parameters
Author: Rainer Meyer, r.meyer494@gmail.com
"""

import math
import numpy as np


TWO_PI = 2*math.pi
PI = math.pi
HALF_PI = math.pi / 2
QUARTER_PI = math.pi / 4
THREE_PI_FOUR = 3 * math.pi / 4

N_REV_JNT = 6
N_LIN_JNT = 1

# Joint max angles (deg, mm)
JOINT_LIMITS = {
    "J1_MAX":   180,
    "J1_MIN":   -180,
    "J2_MAX":   90,
    "J2_MIN":   -80,
    "J3_MAX":   70,
    "J3_MIN":   -80,
    "J4_MAX":   360,
    "J4_MIN":   -360,
    "J5_MAX":   135,
    "J5_MIN":   -100,
    "J6_MAX":   360,
    "J6_MIN":   -360,
    "J7_MAX": 300,
    "J7_MIN": 0,
}

JOINT_LINEAR_LIMITS = {
    'JL1_MAX': 300,
    'JL1_MIN': 0,
    'JL2_MAX': 300,
    'JL2_MIN': 0,
}

# Motor max angles (deg, mm)
MOTOR_LIMITS = {
    "M1_MAX":   180,
    "M1_MIN":   -180,
    "M2_MAX":   90,
    "M2_MIN":   -80,
    "M3_MAX":   35,
    "M3_MIN":   -80,
    "M4_MAX":   360,
    "M4_MIN":   -360,
    "M5_MAX":   135,
    "M5_MIN":   -100,
    "M6_MAX":   360,
    "M6_MIN":   -360,
    "M7_MAX": 300,
    "M7_MIN": 0,
}

MOTOR_LINEAR_LIMITS = {
    'ML1_MAX': 300,
    'ML1_MIN': 0,
    'ML2_MAX': 300,
    'ML2_MIN': 0,
}

# Joint max speed (deg/min, mm/min)
MOTOR_MAX_SPEED = {
    "M1": 3000,
    "M2": 3000,
    "M3": 3000,
    "M4": 5000,
    "M5": 5000,
    "M6": 5000,
    'M7': 3000,
}

MOTOR_LINEAR_MAX_SPEED = {
    "ML1": 3000,
    "ML2": 3000,
}

FRAMES = ("World", "Base", "Tool")

# Denavit-Hartenberg parameters
DH_PARAMS = {
            'a1': 0.030,
            'a2': 0.160,
            'a3': 0.035,
            'd1': 0.130,
            'd4': 0.195,
            'd6': 0.0353}


# Tool frame respect to J6 frame
TOOL_OFS = np.array([
    [1., 0., 0., 0.000],
    [0., 1., 0., 0.000],
    [0., 0., 1., 0.035],
    [0., 0., 0., 1.000]])
# TOOL_OFS[:3, :3] = utils.zyz2rot_mat(np.array([0, -math.pi/4, 0]))

INV_TOOL_OFS = np.linalg.inv(TOOL_OFS)

N_AXIS = 7  # Manipulator axis
N_AXIS_LIN  = 1  # Linear axis

# GUI Defaults
LINEAR_SPEED_MIN = 10  # mm/min
LINEAR_SPEED_DEFAULT = 1200  # mm/min
LINEAR_SPEED_MAX = 4000  # mm/min

ANGULAR_SPEED_MIN = 10  # deg/min
ANGULAR_SPEED_DEFAULT = 1200  # deg/min
ANGULAR_SPEED_MAX = 4000  # deg/min

JOINT_SPEED_MIN = 10  # deg/sec
JOINT_SPEED_DEFAULT = 600  # deg/sec
JOINT_SPEED_MAX = 4000  # deg/sec

LINEAR_INCREMENT = 10  # mm
ANGULAR_INCREMENT = 10  # deg
JOINT_INCREMENT_SCROLL = 1  # deg
JOINT_INCREMENT_ARROW_KEY = 5  # deg


# Reductions (For reference only):
# J1: Planetary: 5:1        Belt: 96:15 = 6.4:1     Total: 32:1
# J2-J6: 30:1
# J7: 8mm / rev