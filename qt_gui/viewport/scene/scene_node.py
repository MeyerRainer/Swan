""" Datastructures for scene tree node.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtGui import QVector3D, QQuaternion, QVector4D
from typing import Optional, Any, Tuple
import numpy as np

from qt_gui.viewport.gizmo.gizmo import Gizmo, HandleType
from qt_gui.viewport.scene.visuals.visual import Renderable
from robot_math.pose import Pose
from robot_math.quaternion import Quaternion


class SceneNode:

    def __init__(self, name: str = "", parent: Optional["SceneNode"] = None, transform: Optional[Pose] = None) -> None:

        self.name: str = name
        self.parent: Optional["SceneNode"] = None
        self.children: list["SceneNode"] = []
        self.visible: bool = True
        self.selectable: bool = True
        self.expanded: bool = False

        self._pose: Pose = transform if transform is not None else Pose.identity()
        self.visual: Optional[Renderable] = None
        self.gizmo: Optional[Gizmo] = None
        self.icon = None

        if parent is not None:
            parent.add_child(self)

    # Tree operations.
    def add_child(self, child: "SceneNode", index: Optional[int] = None) -> None:
        """Adds a child at a specific index or appends to the end."""
        child.parent = self
        if index is None or index < 0 or index >= len(self.children):
            self.children.append(child)
        else:
            self.children.insert(index, child)

    def remove_child(self, child: "SceneNode") -> bool:
        """Removes a child from this node's hierarchy."""
        if child in self.children:
            self.children.remove(child)
            child.parent = None
            return True
        return False

    def child_index(self) -> int:
        """Returns the index of this node relative to its parent."""
        if self.parent and self in self.parent.children:
            return self.parent.children.index(self)
        return 0

    def num_nodes(self) -> int:
        """Recursively counts all descendant nodes plus self."""
        return 1 + sum(child.num_nodes() for child in self.children)

    # --- Transforms & Properties ---
    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, pose) -> None:
        self._pose = pose

    @property
    def position(self) -> QVector3D:
        return QVector3D(*self._pose.position) if self._pose else QVector3D(0, 0, 0)

    @position.setter
    def position(self, pos: QVector3D) -> None:
        # print(f"SceneNode: {self} position set to {pos}")
        self._pose.position = np.array([pos.x(), pos.y(), pos.z()])

    @property
    def quaternion(self) -> QQuaternion:
        return QQuaternion(*self.pose.quaternion.components)

    @quaternion.setter
    def quaternion(self, quat: QQuaternion) -> None:
        q: QVector4D = quat.toVector4D()
        self.pose.quaternion = Quaternion.from_iterable([q.w(), q.x(), q.y(), q.z()])

    # --- Scene Operations ---
    def render(self, context: Any) -> None:
        if not self.visible:
            return
        if self.visual is not None:
            self.visual.render(context, pose=self._pose)
            # print(f"{self.name} render: Pose={self._pose}")
        if self.gizmo is not None:
            self.gizmo.render(context, pose=self._pose)
        for child in self.children:
            child.render(context)

    def ray_hit(self, ray_origin, ray_dir) -> Optional[Tuple[HandleType, Gizmo]]:
        # Return first ray intersection found in children or self
        for child in self.children:
            hit = child.ray_hit(ray_origin, ray_dir)
            if hit is not None:
                return hit
        if self.gizmo is not None:
            return self.gizmo.ray_hit(ray_origin, ray_dir)
        return None