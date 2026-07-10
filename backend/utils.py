"""
Utility functions for Swan application.
Author: Rainer Meyer, rot.meyer494@gmail.com
"""
from math import atan2

from backend.pose import Pose

from typing import List, Tuple
import numpy as np
import numpy.linalg as LA
import math
import re

def mm_min2m_s(mm_min):
    """ Conversion from mm/min to m/s
    @param mm_min: float, speed in millimeters per minute
    """
    return mm_min / 60000

def deg_min2rad_sec(deg_min):
    return deg_min * np.pi / 10800

def quat2rot_mat(q):
    """ Conversion from quaternion to rotation matrix
    @param quat: np.array, quaternion
    """
    q /= np.linalg.norm(q)
    w, x, y, z = q[0], q[1], q[2], q[3]
    return np.array([
        [2*(w*w + x*x) - 1, 2*(x*y - w*z),      2*(x*z + w*y)],
        [2*(x*y + w*z),     2*(w*w + y*y) - 1,  2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),      2*(w*w + z*z) - 1]
    ])

# Conversion from rotation  matrix to quaternion
def rot_mat2quat(R):
    """ Convert a 3x3 rotation matrix to a unit quaternion
    Source: ChatGPT / https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    :param R: (np.ndarray): 3x3 rotation matrix
    :return: quat (np.ndarray): Quaternion [w, x, y, z]
    """
    # Ensure it's a NumPy array
    R = np.array(R, dtype=float)
    trace = np.trace(R)

    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2  # S = 4*w
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S = 4*x
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S = 4*y
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S = 4*z
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S

    # Normalize quaternion to ensure it's a unit quaternion
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)

def rot2zyz(rot_mat: np.ndarray, phi_prev: float = 0, psi_prev: float = 0, flip: bool = False) -> Tuple[np.ndarray, bool]:
    """ Conversion from rotation matrix to ZYZ-Euler angles
    @param rot: np.array, rotation matrix
    @param phi_prev: float, previous value for first Z-rotation.
    Needed in case of a singularity to determine phi and psi (first and last Z-angles)
    @param psi_prev: float, previous value for last Z-rotation
    @param flip: bool. True results in positive Y-rotation, False results in negative Y-rotation. Verify this!
    """
    zyz = np.zeros(3)  # [phi, nu, psi]
    z_i = rot_mat[0][2]  # i-component of Z-unit vector
    z_j = rot_mat[1][2]  # j-component of Z-unit vector
    z_hypo = math.sqrt(z_i*z_i + z_j*z_j)

    # Singularity if Z is vertical -> phi and psi dependent
    eps = 1e-3
    if z_hypo < eps:  # Singularity
        zyz[0] = phi_prev  # Force joint 4 to have value of last non-singular posture

        zyz[1] = 0  # Joint 5 limits allow only for this singularity to happen

        # Total rotation, sum of phi and psi
        gamma = atan2(rot_mat[1][0], rot_mat[0][0])

        # Solve phi and psi as least squares solution
        zyz[0] = 0.5 * (gamma + phi_prev - psi_prev)  # Phi / J4
        zyz[2] = 0.5 * (gamma - phi_prev + psi_prev)  # Psi / J6

        return zyz, False

    # Fully defined rotation: Nu = [0, pi] or [-pi, 0] when flipped
    if flip:
        phi = math.atan2(-z_j, -z_i)
        nu = math.atan2(-z_hypo, rot_mat[2][2])
        psi = math.atan2(-rot_mat[2][1], rot_mat[2][0])
    else:
        phi = math.atan2(z_j, z_i)
        nu = math.atan2(z_hypo, rot_mat[2][2])
        psi = math.atan2(rot_mat[2][1], -rot_mat[2][0])

    zyz[0] = phi  # Around Z
    zyz[1] = nu  # Around new Y
    zyz[2] = psi  # Around new Z

    return zyz, True

def zyz2rot_mat(zyz, degrees=False):
    """ Conversion from ZYZ-Euler angles  to a rotation matrix
    @param zyz: np.array, ZYZ-angles
    @param degrees: bool, degrees or radians
    """
    if degrees:
        zyz = np.deg2rad(zyz)

    z1, y1, z2 = zyz[0], zyz[1], zyz[2]

    rot_alpha = np.array([
        [math.cos(z1), -math.sin(z1), 0.],
        [math.sin(z1), math.cos(z1), 0.],
        [0., 0., 1.]])

    rot_nu = np.array([
        [math.cos(y1), 0, math.sin(y1)],
        [0, 1, 0],
        [-math.sin(y1), 0, math.cos(y1)]])

    rot_psi = np.array([
        [math.cos(z2), -math.sin(z2), 0],
        [math.sin(z2), math.cos(z2), 0],
        [0, 0, 1]])

    return rot_alpha @ rot_nu @ rot_psi

def zyz2quat(zyz, degrees=False):
    """ Conversion from ZYZ-Euler angles to quaternion
    @param zyz: np.array, ZYZ-angles
    @param degrees: bool, degrees or radians
    """
    return rot_mat2quat(zyz2rot_mat(zyz, degrees=degrees))

def rot_x_quat(theta, degrees=False):
    """ Quaternion from X-Rotation
    @param theta: float, rotation angle around X-axis
    @param degrees: bool, degrees or radians
    """
    if degrees:
        theta = np.radians(theta)
    half_theta = theta / 2
    return np.array([np.cos(half_theta), np.sin(half_theta), 0.0, 0.0])  # [w, x, y, z]

def rot_y_quat(theta, degrees=False):
    """ Quaternion from Y-Rotation
    @param theta: float, rotation angle around Y-axis
    @param degrees: bool, degrees or radians
    """
    if degrees:
        theta = np.radians(theta)
    half_theta = theta / 2
    return np.array([np.cos(half_theta), 0.0, np.sin(half_theta), 0.0])  # [w, x, y, z]

def rot_z_quat(theta, degrees=False):
    """ Quaternion from Z-Rotation
    @param theta: float, rotation angle around Z-axis
    @param degrees: bool, degrees or radians
    """
    if degrees:
        theta = np.radians(theta)
    half_theta = theta / 2
    return np.array([np.cos(half_theta), 0.0, 0.0, np.sin(half_theta)])  # [w, x, y, z]


def slerp(a, b, t):
    """ Spherical linear interpolation  for generating smooth rotations between two quaternions
    @param a: np.array, first quaternion
    @param b: np.array, second quaternion
    @param t: float in range [0, 1], interpolation variable
    """
    similarity = np.dot(a, b)
    similarity = np.clip(similarity, -1.0, 1.0)

    # For the shorter rotation path, choose the quaternion-vector pointing in similar direction
    if similarity < 0.0:
        b = -b
        similarity = -similarity

    # If the start and end orientations are too similar, slerping results in zero-denominator.
    # For small orientation changes, linearly interpolate between quaternions instead.
    dot_product_threshold = 0.9995
    if similarity > dot_product_threshold:
        # Use LERP and normalize result
        result = a + t*(b - a)
        return result / np.linalg.norm(result)
    # For non-similar orientation, compute spherical interpolation (Shortest path on the surface of R^4 unit sphere).
    else:
        theta = math.acos(similarity)
        result = (math.sin((1-t)*theta) / math.sin(theta))*a + (math.sin(t*theta) / math.sin(theta))*b
        return result / np.linalg.norm(result)

def dir_vec_angle2quat(dir_vec: np.ndarray, angle: float, degrees=False):
    if degrees:
        angle = np.radians(angle)

    norm = np.linalg.norm(dir_vec)

    if norm < 1e-10:
        raise ValueError("Rotation axis must be non-zero")

    unit_vec = dir_vec / norm
    half_angle = angle / 2
    quat = np.array((np.cos(half_angle), 0., 0., 0.))
    quat[1:] = np.sin(half_angle) * unit_vec

    return quat

# Quaternion multiplication
def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiplies two quaternions q1 * q2 and returns a normalized quaternion.
    Both inputs and output are in the format [w, x, y, z].
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2

    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)

def parse_grbl_status(line: str):
    """ Parses GRBL status line
    @param line: GRBL status line
    @return: Tuple[str, list[float], list[float]]
    """
    s_match = re.search(r"<([a-zA-Z]+)", line)
    m_match = re.search(r"MPos:(.*?),WPos:", line)
    w_match = re.search(r"WPos:(.*?)>", line)

    if s_match:
        status = s_match.group(1)
    else:
        status = "Parser error"
    m_pos = [float(x) for x in m_match.group(1).split(",")] if m_match else []
    w_pos = [float(x) for x in w_match.group(1).split(",")] if w_match else []

    return status, m_pos, w_pos

def pose_interpolator(a: Pose, b: Pose, segment_size_m: float = 0.001, segment_size_rad: float = 0.0035, segment_count: int = None) -> list[Pose] | None:

    poses: List[Pose] = []

    if a.is_close(b):
        return None

    start_pos = a.position
    start_quat = a.quaternion
    end_pos = b.position
    end_quat = b.quaternion

    # Translational error, meters
    transl = end_pos - start_pos
    transl_norm = LA.norm(transl)
    n_segments_lin = int(np.ceil(transl_norm / segment_size_m))

    if segment_count is not None:
        n_segments = segment_count
    else:
        # Rotational error, radians
        similarity = np.clip(np.dot(start_quat, end_quat), -1, 1)
        if similarity < 0:
            end_quat *= -1
            similarity *= -1
        rotation_dist = 2 * math.acos(similarity)
        n_segments_ang = int(np.ceil(rotation_dist / segment_size_rad))

        # Choose whether rotation or translation determines segment count
        n_segments = max(n_segments_lin, n_segments_ang)

    for idx in range(n_segments):
        t = (idx + 1) / n_segments  # Interpolation parameter in range ]0, 1]
        pose_interp: Pose = Pose.identity()
        pose_interp.position = start_pos + t * transl  # Interpolated position
        pose_interp.quaternion = slerp(start_quat, end_quat, t)  # Interp. orientation
        poses.append(pose_interp)

    return poses
