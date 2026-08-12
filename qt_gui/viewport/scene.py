""" Datastructures for scene tree.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtCore import Qt, QModelIndex, QAbstractItemModel, QIODevice, QDataStream, QMimeData, QByteArray
from PyQt6.QtGui import QVector3D, QQuaternion, QVector4D
from typing_extensions import override
from typing import Optional, Any
from pathlib import Path
import numpy as np

from qt_gui.viewport.shapes import MeshObject, MeshSpecs
from qt_gui.viewport.visuals.ellipsoid import EllipsoidSpecs, Ellipsoid
from qt_gui.viewport.visuals.gizmo import Gizmo
from qt_gui.viewport.visuals.grid import Grid, GridSpecs
from qt_gui.viewport.visuals.visual import Renderable
from robot_math.pose import Pose
from robot_math.quaternion import Quaternion


class SceneNode:

    def __init__(self, name: str = "", parent: Optional["SceneNode"] = None) -> None:

        self.name: str = name
        self.parent: Optional["SceneNode"] = None
        self.children: list["SceneNode"] = []
        self.visible: bool = True
        self.selectable: bool = True
        self.expanded: bool = False

        self._pose: Pose = Pose.identity()
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
            self.visual.render(context, self._pose)
        if self.gizmo is not None:
            self.gizmo.render(context, self._pose)
        for child in self.children:
            child.render(context)

    def ray_hit(self, ray_origin, ray_dir) -> Optional[Any]:
        # Return first ray intersection found in children or self
        for child in self.children:
            hit = child.ray_hit(ray_origin, ray_dir)
            if hit is not None:
                return hit, child.gizmo
        if self.gizmo is not None:
            return self.gizmo.ray_hit(ray_origin, ray_dir)
        return None


class SceneGraph(QAbstractItemModel):

    MIME_TYPE = "application/x-scenenode-pointer"

    def __init__(self, root: Optional[SceneNode] = None, parent=None, dir_path: str = ""):

        super().__init__(parent)

        self.root_node = root or SceneNode("Root")
        if dir_path:
            self.build_from_directory(dir_path)

        grid_node = SceneNode(name="Grid", parent=self.root_node)
        grid_node.visual = Grid(name="Grid", params=GridSpecs())
        self.add_node(parent_idx=QModelIndex(), node=grid_node)

        ellipsoid_node = SceneNode(name="Ellipsoid", parent=self.root_node)
        ellipsoid_node.visual = Ellipsoid(EllipsoidSpecs(radii=(0.05, 0.02, 0.01)))
        ellipsoid_node.gizmo = Gizmo(ellipsoid_node)
        self.add_node(parent_idx=QModelIndex(), node=grid_node)

    def size(self):
        return self.root_node.num_nodes()

    @property
    def root(self) -> SceneNode:
        return self.root_node

    def node_from_index(self, index: QModelIndex) -> SceneNode:
        if index.isValid():
            return index.internalPointer()
        return self.root_node

    def index_from_node(self, node: SceneNode) -> QModelIndex:
        """Constructs a QModelIndex from a SceneNode pointer."""
        if node is self.root_node or node.parent is None:
            return QModelIndex()
        return self.createIndex(node.child_index(), 0, node)

    # --- Model Structure Overrides ---
    @override
    def rowCount(self, parent: QModelIndex = QModelIndex()) -> int:
        if parent.column() > 0:
            return 0
        parent_node = self.node_from_index(parent)
        return len(parent_node.children)

    @override
    def columnCount(self, parent: QModelIndex = QModelIndex()) -> int:
        return 1

    @override
    def index(self, row: int, column: int, parent: QModelIndex = QModelIndex()) -> QModelIndex:
        if not self.hasIndex(row, column, parent):
            return QModelIndex()

        parent_node = self.node_from_index(parent)
        if 0 <= row < len(parent_node.children):
            return self.createIndex(row, column, parent_node.children[row])
        return QModelIndex()

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

        if role in (Qt.ItemDataRole.DisplayRole, Qt.ItemDataRole.EditRole):
            return node.name
        elif role == Qt.ItemDataRole.DecorationRole:
            return node.icon

        return None

    @override
    def setData(self, index: QModelIndex, value: Any, role: int = Qt.ItemDataRole.EditRole) -> bool:
        if index.isValid() and role == Qt.ItemDataRole.EditRole:
            node = self.node_from_index(index)
            node.name = str(value)
            self.dataChanged.emit(index, index, [Qt.ItemDataRole.DisplayRole])
            return True
        return False

    @override
    def flags(self, index: QModelIndex) -> Qt.ItemFlag:
        if not index.isValid():
            # Allow dropping onto root empty space
            return Qt.ItemFlag.ItemIsDropEnabled

        node = self.node_from_index(index)
        flags = Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsEditable | Qt.ItemFlag.ItemIsDragEnabled | Qt.ItemFlag.ItemIsDropEnabled

        if node.selectable:
            flags |= Qt.ItemFlag.ItemIsSelectable

        return flags

    # --- Dynamic Interactivity (Add, Remove, Drag-and-Drop) ---
    def add_node(self, parent_idx: QModelIndex, node: SceneNode, row: Optional[int] = None) -> QModelIndex:
        parent_node = self.node_from_index(parent_idx)
        insert_row = len(parent_node.children) if row is None else row

        self.beginInsertRows(parent_idx, insert_row, insert_row)
        parent_node.add_child(node, insert_row)
        self.endInsertRows()

        return self.index(insert_row, 0, parent_idx)

    def remove_node(self, index: QModelIndex) -> bool:
        if not index.isValid():
            return False

        child_node = self.node_from_index(index)
        parent_node = child_node.parent or self.root_node
        row = index.row()

        self.beginRemoveRows(self.parent(index), row, row)
        parent_node.remove_child(child_node)
        self.endRemoveRows()
        return True

    def update(self):
        # TODO
        pass

    # --- Drag & Drop Support (Reordering / Moving Nodes) ---
    @override
    def supportedDropActions(self) -> Qt.DropAction:
        return Qt.DropAction.MoveAction

    @override
    def moveRows(self, sourceParent: QModelIndex, sourceRow: int, count: int, destinationParent: QModelIndex, destinationChild: int) -> bool:
        if count != 1:
            return False  # Supporting single-node move for simplicity

        src_node = self.node_from_index(sourceParent)
        dst_node = self.node_from_index(destinationParent)

        if sourceRow >= len(src_node.children):
            return False

        # Adjust target index for intra-parent moves
        target_idx = destinationChild
        if sourceParent == destinationParent and destinationChild > sourceRow:
            target_idx -= 1

        # Prevent dropping a parent inside its own subtree
        moved_node = src_node.children[sourceRow]
        curr = dst_node
        while curr is not None:
            if curr == moved_node:
                return False
            curr = curr.parent

        if not self.beginMoveRows(sourceParent, sourceRow, sourceRow, destinationParent, destinationChild):
            return False

        # Update underlying tree structure
        node_to_move = src_node.children.pop(sourceRow)
        dst_node.add_child(node_to_move, target_idx)

        self.endMoveRows()
        return True

    def build_from_directory(self, dir_path: str | Path):
        path = Path(dir_path)
        if not path.exists() or not path.is_dir():
            raise ValueError(f"Invalid directory path: {dir_path}")

        self.beginResetModel()
        self.root_node = SceneNode(path.name)
        self._populate_directory_tree(path, self.root_node)
        self.endResetModel()

    def _populate_directory_tree(self, current_path: Path, parent_node: SceneNode):
        for entry in sorted(current_path.iterdir()):
            if entry.is_dir():
                dir_node = SceneNode(entry.name, parent=parent_node)
                self._populate_directory_tree(entry, dir_node)
            elif entry.suffix.lower() == ".obj":
                new_node: SceneNode = SceneNode(name=entry.name, parent=parent_node)
                mesh_params = MeshSpecs(file_path=entry)
                new_node.visual = MeshObject(mesh_params)

    # --- Drag & Drop MIME Handlers ---

    @override
    def mimeTypes(self) -> list[str]:
        return [self.MIME_TYPE]

    @override
    def mimeData(self, indexes: list[QModelIndex]) -> QMimeData:
        """Packs the Python object pointer address into the drag payload."""
        mime_data = QMimeData()
        encoded_data = QByteArray()
        stream = QDataStream(encoded_data, QIODevice.OpenModeFlag.WriteOnly)

        for index in indexes:
            if index.isValid() and index.column() == 0:
                node = self.node_from_index(index)
                # Store memory address of the SceneNode python object
                stream.writeUInt64(id(node))

        mime_data.setData(self.MIME_TYPE, encoded_data)
        return mime_data

    @override
    def dropMimeData(self, data: QMimeData, action: Qt.DropAction, row: int, column: int,
                     parent: QModelIndex) -> bool:
        """Unpacks the node pointer and handles structural reparenting."""
        if action == Qt.DropAction.IgnoreAction:
            return True

        if not data.hasFormat(self.MIME_TYPE):
            return False

        encoded_data = data.data(self.MIME_TYPE)
        stream = QDataStream(encoded_data, QIODevice.OpenModeFlag.ReadOnly)

        node_ptr = stream.readUInt64()
        if not node_ptr:
            return False

        # Retrieve python object from address pointer
        import ctypes
        dragged_node: SceneNode = ctypes.cast(node_ptr, ctypes.py_object).value

        # Calculate target parent & destination index
        target_parent_node = self.node_from_index(parent)

        # If dropped directly onto an item (row == -1), append to its children
        if row == -1:
            destination_row = len(target_parent_node.children)
        else:
            destination_row = row

        # Prevent dropping a parent into its own children subtree
        curr = target_parent_node
        while curr is not None:
            if curr == dragged_node:
                return False
            curr = curr.parent

        # Perform movement
        source_parent_node = dragged_node.parent or self.root_node
        source_row = dragged_node.child_index()

        # Index for source.
        # source_parent_idx = self.createIndex(source_parent_node.child_index(), 0, source_parent_node) if source_parent_node != self.root_node else QModelIndex()
        if source_parent_node == self.root_node:
            source_parent_idx = QModelIndex()
        else:
            source_parent_idx = self.createIndex(source_parent_node.child_index(), 0, source_parent_node)


        # Adjust index shift if moving within the exact same parent
        if source_parent_node == target_parent_node and destination_row > source_row:
            adjusted_destination = destination_row
        else:
            adjusted_destination = destination_row

        if not self.beginMoveRows(source_parent_idx, source_row, source_row, parent, adjusted_destination):
            return False

        # Update underlying python references
        source_parent_node.remove_child(dragged_node)

        # Recalculate target row in case array shifted after removal
        if source_parent_node == target_parent_node and source_row < destination_row:
            destination_row -= 1

        target_parent_node.add_child(dragged_node, destination_row)
        self.endMoveRows()

        return True
