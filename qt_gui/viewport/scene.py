""" Datastructures for scene tree.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

from __future__ import annotations

from qt_gui.viewport.shapes import ArrowSpecs, RingSpecs, PlaneSpecs, RotationRing, DragPlane, DragArrow
from robot_math.pose import Pose
from qt_gui.viewport.visuals.visual import Visual, Material, load_obj_file
from qt_gui.viewport.visuals.grid import Grid, GridSpecs

from pathlib import Path
from typing import override, Optional
from PyQt6.QtCore import QAbstractItemModel, QModelIndex, Qt
from PyQt6.QtGui import QIcon
from dataclasses import dataclass, field


class SceneNode:

    def __init__(self, name: str = "", parent: "SceneNode | None" = None):

        self.name: str = name
        self.parent: SceneNode | None = parent
        self.children: list[SceneNode] = []
        self.visible: bool = True
        self.selectable: bool = True
        self.expanded: bool = False

        self.pose: Pose = Pose.identity()
        self.visual: Optional[Visual] = None
        self.icon = None

        if parent is not None:
            parent.add_child(self)

        # print(f"Node initialized")

    def add_child(self, child: SceneNode):
        child.parent = self
        self.children.append(child)

    def child_index(self) -> int:
        """Returns the index of this node relative to its parent."""
        if self.parent:
            return self.parent.children.index(self)
        return 0

class Frame(SceneNode):

    def __init__(self):

        super().__init__()

        self.x_axis_specs = ArrowSpecs()
        self.visual = DragArrow(self.x_axis_specs)
        # self.ring_specs = RingSpecs()
        # self.visual = RotationRing(self.ring_specs)
        # self.plane_specs = PlaneSpecs()
        # self.visual = DragPlane(self.plane_specs)



class GridNode(SceneNode):

    def __init__(self):

        super().__init__()

        self.visual = Grid(name="Grid", params=GridSpecs())
        self.selectable = False


class SceneGraph(QAbstractItemModel):

    def __init__(self, root: SceneNode | None = None, parent=None, dir_path: str = ""):

        super().__init__(parent)

        self.root_node = root or SceneNode("Root")

        self._build_from_directory(dir_path)
        self.root_node.add_child(GridNode())
        self.root_node.add_child(Frame())

    @property
    def root(self):
        return self.root_node

    def _build_from_directory(self, dir_path: str | Path):
        """Rebuilds the scene graph from a file system directory containing .obj models."""
        path = Path(dir_path)
        if not path.exists() or not path.is_dir():
            raise ValueError(f"Invalid directory path: {dir_path}")

        # Signal model reset to QTreeView
        self.beginResetModel()

        self.root_node = SceneNode(path.name)
        self._populate_directory_tree(path, self.root_node)

        self.endResetModel()

    def _populate_directory_tree(self, current_path: Path, parent_node: SceneNode):
        for entry in sorted(current_path.iterdir()):
            if entry.is_dir():
                # Folder node
                dir_node = SceneNode(entry.name, parent=parent_node)
                self._populate_directory_tree(entry, dir_node)

            elif entry.suffix.lower() == ".obj":
                # OBJ Model node
                obj_node = SceneNode(entry.name, parent=parent_node)
                visuals = load_obj_file(entry)

                if len(visuals) == 1:
                    # Single mesh OBJ
                    obj_node.visual = visuals[0]
                elif len(visuals) > 1:
                    # Multi-shape OBJ: create child sub-mesh nodes
                    for idx, vis in enumerate(visuals):
                        sub_node = SceneNode(f"Mesh_{idx}", parent=obj_node)
                        sub_node.visual = vis

    def node_from_index(self, index: QModelIndex) -> SceneNode:
        """ Helper to extract SceneNode from QModelIndex.
        :param index: ?
        """
        if index.isValid():
            return index.internalPointer()
        return self.root_node

    @override
    def rowCount(self, parent: QModelIndex = QModelIndex()) -> int:
        if parent.column() > 0:
            return 0
        parent_node = self.node_from_index(parent)
        return len(parent_node.children)

    @override
    def columnCount(self, parent: QModelIndex = QModelIndex()) -> int:
        return 1  # Expand if you want columns for Pose, Visibility, etc.

    @override
    def index(self, row: int, column: int, parent: QModelIndex = QModelIndex()) -> QModelIndex:
        if not self.hasIndex(row, column, parent):
            return QModelIndex()

        parent_node = self.node_from_index(parent)
        child_node = parent_node.children[row]

        # Pointer to actual native object passed as internalPointer
        return self.createIndex(row, column, child_node)

    @override
    def parent(self, index: QModelIndex) -> QModelIndex:
        if not index.isValid():
            return QModelIndex()

        child_node = self.node_from_index(index)
        parent_node = child_node.parent

        if parent_node is None or parent_node == self.root_node:
            return QModelIndex()

        return self.createIndex(parent_node.child_index(), 0, parent_node)

    @override
    def data(self, index: QModelIndex, role: int = Qt.ItemDataRole.DisplayRole):
        if not index.isValid():
            return None

        node = self.node_from_index(index)

        if role == Qt.ItemDataRole.DisplayRole:
            return node.name

        elif role == Qt.ItemDataRole.DecorationRole:
            return node.icon  # Qt automatically handles QIcon rendering

        return None

    @override
    def flags(self, index: QModelIndex) -> Qt.ItemFlag:
        if not index.isValid():
            return Qt.ItemFlag.NoItemFlags

        node = self.node_from_index(index)
        flags = Qt.ItemFlag.ItemIsEnabled

        if node.selectable:
            flags |= Qt.ItemFlag.ItemIsSelectable

        return flags
