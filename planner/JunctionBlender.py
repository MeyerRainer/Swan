from typing import Tuple, List

from robot_math.pose import Pose
import numpy as np


class JunctionBlender:

    @staticmethod
    def blend_junction(p1: Pose, p2: Pose, p3: Pose, zone: float) -> Tuple[Pose, Pose, List[Pose]]:
        """ Blends the corner at p2 based on the previous and next target.
        :param p1: Pose 1
        :param p2: Pose 2
        :param p3: Pose 3
        :param zone:
        :return: True if segments created.
        """
        # Build a list of poses to construct a smooth path: p1->p2->p3
        u: np.ndarray = p2.position - p1.position  # p1 -> p2
        v: np.ndarray = p3.position - p2.position  # p2 -> p3
        u_norm: float = np.linalg.norm(u)
        v_norm: float = np.linalg.norm(v)
        u_normalized = u / u_norm  # Unit vec
        v_normalized = v / v_norm  # Unit vec

        # Stop at p2 or start/end coincident with junction point, no junction to blend.
        if u_norm < 1e-7 or v_norm < 1e-7 or zone <= 0:
            return p1, p2, []

        # Cosine of full junction angle. -1 at U-turn, 0 at right angle junction, 1 at straight path.
        cos_two_alpha: float = np.clip(np.dot(u_normalized, v_normalized), -1, 1)
        # Path straight enough, no junction to blend.
        if cos_two_alpha > 0.999:
            return p1, p3, []
        # Alpha is junction half angle (complete U-turn = 0, straight path with no junction = pi/2).
        alpha: float = 0.5 * np.acos(cos_two_alpha)

        # Blend radius based on zone
        radius: float = zone / (1 / np.sin(alpha) - 1)

        # p2's distance from blending arc's start and end points.
        cut_distance: float = (radius + zone) * np.sin(np.pi / 2 - alpha)

        # Clamp cut distance to half target distance to avoid overlapping to adjacent segments.
        # cut_distance = min(min(0.5*u_norm, 0.5*v_norm), cut_distance)

        # Poses at blending arc start and end.
        arc_begin: Pose = p1.interpolate(p2, t = 1 - (cut_distance / u_norm))
        arc_end: Pose = p2.interpolate(p3, t = cut_distance / v_norm)

        return p1, p2, [arc_begin, arc_end]
        # if is_collinear_by_deviation(p1.position, p2.position, p3.position, max_deviation=0.0001):
