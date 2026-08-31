"""
Configuration file for parameters

Author: Rainer Meyer, r.meyer494@gmail.com
"""

import math
from typing import List

import numpy as np

# Linear base DOF's
LINEAR_AXIS = ['X']

N_REV_JNT = 6
N_LIN_JNT = len(LINEAR_AXIS)
N_JNT = N_REV_JNT + N_LIN_JNT


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
    "J7_MAX": 0,
    "J7_MIN": 0,
}

JOINT_LINEAR_LIMITS = {
    'JL1_MAX': 200,
    'JL1_MIN': -200,
    'JL2_MAX': 0,
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
    "M7_MAX": 0,
    "M7_MIN": 0,
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
DH_TABLE: List[dict] = [
    {'a': np.float64(0.030), 'alpha': np.float64(math.pi/2),    'd': np.float64(0.130), 'nu_offset': np.float64(0.000)},
    {'a': np.float64(0.160), 'alpha': np.float64(0.000),        'd': np.float64(0.000), 'nu_offset': np.float64(math.pi/2)},
    {'a': np.float64(0.035), 'alpha': np.float64(math.pi/2),    'd': np.float64(0.000), 'nu_offset': np.float64(0.000)},
    {'a': np.float64(0.000), 'alpha': np.float64(-math.pi/2),   'd': np.float64(0.195), 'nu_offset': np.float64(0.000)},
    {'a': np.float64(0.000), 'alpha': np.float64(math.pi/2),    'd': np.float64(0.000), 'nu_offset': np.float64(0.000)},
    {'a': np.float64(0.000), 'alpha': np.float64(0.000),        'd': np.float64(0.0353), 'nu_offset': np.float64(0.000)},
]

# Spring geometry
SHOULDER_TO_SPRING = np.float64(0.040)
ELBOW_TO_SPRING = np.float64(0.025)
SPRING_CENTER_DIST = np.float64(0.056)
L2_LENGTH = np.float64(0.160)

PARALLEL_LINK_DIST = np.float64(0.090)

CHAR_LEN = np.float64(0.2)  # Characteristic length, m

# Sled total length (X-axis): 150mm
# Sled rail center-to-center width (Y-axis): 118mm
BASE_OFFSET = np.array([0.225, 0.380, 0.0], dtype=np.float64)  # Offset from world origin to base at zero linear joints.

# Tool frame respect to J6 frame
TOOL_OFS = np.array([
    [1., 0., 0., 0.000],
    [0., 1., 0., 0.000],
    [0., 0., 1., 0.000],
    [0., 0., 0., 1.000]], dtype=np.float64)
# TOOL_OFS[:3, :3] = utils.zyz2rot_mat(np.array([0, -math.pi/4, 0]), dtype=np.float64)
INV_TOOL_OFS = np.linalg.inv(TOOL_OFS)

# Calibration board w.r.t. world frame. (World to cal.board. transformation)
WORLD2CAL_BOARD = np.array([
    [1., 0., 0., 0.420],
    [0., -1., 0., 0.140],
    [0., 0., -1., 0.001],
    [0., 0., 0., 1.]], dtype=np.float64)
CAL_BOARD2WORLD = np.linalg.inv(WORLD2CAL_BOARD)

# GUI Defaults
LINEAR_SPEED_MIN: int = 10  # mm/min
LINEAR_SPEED_DEFAULT: int = 1200  # mm/min
LINEAR_SPEED_MAX: int = 4000  # mm/min

ANGULAR_SPEED_MIN: int = 10  # deg/min
ANGULAR_SPEED_DEFAULT: int = 1200  # deg/min
ANGULAR_SPEED_MAX: int = 4000  # deg/min

JOINT_SPEED_MIN: int = 10  # deg/sec
JOINT_SPEED_DEFAULT: int = 600  # deg/sec
JOINT_SPEED_MAX: int = 4000  # deg/sec

LINEAR_INCREMENT: int = 10  # mm
ANGULAR_INCREMENT: int = 10  # deg
JOINT_INCREMENT_SCROLL: int = 1  # deg
JOINT_INCREMENT_ARROW_KEY: int = 5  # deg

SPRING_MOUNT_DIMENSIONS = {
    "MAIN_BOTTOM": 0.040,
    "MAIN_TOP": 0.025,
}


# TODO: Move somewhere else.
# Reductions (For reference only):
# J1: Planetary: 5:1        Belt: 96:15 = 6.4:1     Total: 32:1
# J2-J6: 30:1
# J7: 8mm / rev

# Default height: d1 + a2 + a3 = 0.325
# Default length: a1 + d4 + d6 <=> 0.30+0.190+0.030=0.250