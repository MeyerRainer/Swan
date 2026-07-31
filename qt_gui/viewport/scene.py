""" Datastructures for scene tree.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

from __future__ import annotations

from typing import override

from PyQt6.QtCore import QAbstractItemModel, QModelIndex, Qt
from PyQt6.QtGui import QIcon

from robot_math.pose import Pose
from qt_gui.viewport.visuals.visual import Visual


class SceneNode:

    def __init__(self, name: str = "", parent: "SceneNode | None" = None):

        self.name: str = name
        self.parent: SceneNode | None = parent
        self.children: list[SceneNode] = []

        self.visible: bool = True
        self.selectable: bool = True
        self.expanded: bool = False

        self.pose = None  # Replace with your Pose object
        self.visual = None  # Replace with your Visual object
        self.icon = None  # QIcon | None

        if parent is not None:
            parent.add_child(self)

    def add_child(self, child: SceneNode):
        child.parent = self
        self.children.append(child)

    def child_index(self) -> int:
        """Returns the index of this node relative to its parent."""
        if self.parent:
            return self.parent.children.index(self)
        return 0


class Scene:

    def __init__(self):

        self.root = SceneNode("Scene")


class SceneTreeModel(QAbstractItemModel):

    def __init__(self, root: SceneNode, parent=None):

        super().__init__(parent)
        self.root_node = root

    def node_from_index(self, index: QModelIndex) -> SceneNode:
        """Helper to extract SceneNode from QModelIndex."""
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
