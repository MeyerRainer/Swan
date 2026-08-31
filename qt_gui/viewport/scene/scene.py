""" Datastructures for scene tree.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtCore import Qt, QModelIndex, QAbstractItemModel, QIODevice, QDataStream, QMimeData, QByteArray
from typing_extensions import override
from typing import Optional, Any, Dict
from pathlib import Path

import config
from robot_manipulator.robot_system import RobotSystem
from qt_gui.viewport.scene.visuals.mesh import MeshObject, MeshSpecs
from qt_gui.viewport.scene.visuals.ellipsoid import EllipsoidSpecs, Ellipsoid
from qt_gui.viewport.gizmo.gizmo import Gizmo
from qt_gui.viewport.scene.visuals.grid import Grid, GridSpecs
from qt_gui.viewport.scene.scene_node import SceneNode
from robot_math.pose import Pose


class SceneGraph(QAbstractItemModel):

    MIME_TYPE = "application/x-scenenode-pointer"

    def __init__(self, root: Optional[SceneNode] = None, parent=None, dir_path: str = ""):

        super().__init__(parent)

        self.root_node = root or SceneNode("Root")

        self._robot_link_nodes: Optional[Dict[str, SceneNode]] = {}  # SceneNodes for robot links
        self._robot_spring_nodes: Optional[Dict[str, SceneNode]] = {}
        self._robot_counter_weight_node: Optional[SceneNode] = None

        if dir_path:
            self.build_from_directory(dir_path)

        self.L0 = SceneNode(name="L0", parent=self.root_node)
        self.L0.visual = MeshObject(MeshSpecs(file_path=Path("scene/robot/L0.obj")))

        grid_node = SceneNode(name="Grid", parent=self.root_node)
        grid_node.visual = Grid(name="Grid", params=GridSpecs())
        # self.add_node(parent_idx=QModelIndex(), node=grid_node)

        ellipsoid_node = SceneNode(name="Ellipsoid", parent=self.root_node)
        ellipsoid_node.visual = Ellipsoid(EllipsoidSpecs(radii=(0.05, 0.02, 0.01)))
        ellipsoid_node.gizmo = Gizmo(ellipsoid_node)
        # self.add_node(parent_idx=QModelIndex(), node=ellipsoid_node)

    def update_robot_sys(self, robot_sys: RobotSystem):
        link_poses = robot_sys.sys_state.mcu.manipulator.link_poses
        spring_poses = robot_sys.sys_state.mcu.manipulator.spring_poses
        cw_pose = robot_sys.sys_state.mcu.manipulator.counter_weight_pose
        pl_pose = robot_sys.sys_state.mcu.manipulator.parallel_link_pose
        base_pose: Pose = robot_sys.sys_state.mcu.linear_axis.pose
        if "L1" in self._robot_link_nodes:
            self._robot_link_nodes["L1"].pose = base_pose
        if "L2" in self._robot_link_nodes:
            self._robot_link_nodes["L2"].pose = base_pose.compose(link_poses[0])
        if "L3" in self._robot_link_nodes:
            self._robot_link_nodes["L3"].pose = base_pose.compose(link_poses[1])
        if "L4" in self._robot_link_nodes:
            self._robot_link_nodes["L4"].pose = base_pose.compose(link_poses[2])
        if "L5" in self._robot_link_nodes:
            self._robot_link_nodes["L5"].pose = base_pose.compose(link_poses[3])
        if "L6" in self._robot_link_nodes:
            self._robot_link_nodes["L6"].pose = base_pose.compose(link_poses[4])
        if "ToolFrame" in self._robot_link_nodes:
            self._robot_link_nodes["ToolFrame"].pose = base_pose.compose(link_poses[5].compose(Pose(config.TOOL_OFS)))
        if "LCW" in self._robot_link_nodes:
            self._robot_link_nodes["LCW"].pose = base_pose.compose(cw_pose)
        if "LPL" in self._robot_link_nodes:
            self._robot_link_nodes["LPL"].pose = base_pose.compose(pl_pose)
        if "SpringDown" in self._robot_link_nodes:
            self._robot_link_nodes["SpringDown"].pose = base_pose.compose(spring_poses[0])

        # self._robot_spring_nodes["left_down"].pose = sprint_poses[0]
        # self._robot_spring_nodes["right_down"].pose = sprint_poses[1]
        # self._robot_spring_nodes["left_up"].pose = sprint_poses[2]
        # self._robot_spring_nodes["right_up"].pose = sprint_poses[3]
        #
        # self._robot_counter_weight_node.pose = cw_pose

    def size(self):
        return self.root_node.num_nodes()

    def render(self, render_context) -> None:
        # Recursively render from root.
        self.root_node.render(render_context)

    def ray_hit(self, ray_origin, ray_dir) -> Optional[Any]:
        # Recursively call ray_hit from root.
        return self.root_node.ray_hit(ray_origin, ray_dir)

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

    def print_tree(self, node: Optional[SceneNode] = None, indent: int = 1) -> None:
        """Recursively print the scene graph hierarchy for debugging."""
        if node is None:
            node = self.root_node

        prefix = "    " * indent
        pose = node.pose

        print(f"{prefix}├─ {node.name or '<unnamed>'} [visible={node.visible}, "
            f"visual={'Y' if node.visual else 'N'}, "
            f"gizmo={'Y' if node.gizmo else 'N'}]")

        print(f"{prefix}│  Position={pose.position}, Euler={pose.zyz_euler}")
        print(f"{prefix}│")

        for child in node.children:
            self.print_tree(child, indent + 1)

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
                # if entry.name == "robot":
                #     self._set_robot_visuals(entry)
                # else:
                self._populate_directory_tree(entry, dir_node)
            elif entry.suffix.lower() == ".obj":
                new_node: SceneNode = SceneNode(name=entry.name, parent=parent_node)
                mesh_params = MeshSpecs(file_path=entry)
                new_node.visual = MeshObject(mesh_params)
                # If new node is a robot link, take a reference to it.
                if entry.parent.name == "robot":
                    link_name: str = entry.name.split(".")[0]
                    self._robot_link_nodes[link_name] = new_node



    # def _set_robot_visuals(self, current_path: Path) -> None:
    #     """ Build robot using links in the "robot" folder and DH-table in config.
    #     """
    #     # Set meshes for links
    #     for entry in sorted(current_path.iterdir()):
    #         if entry.is_dir():
    #             print(entry)
    #             raise ValueError("Robot directory should only contain .obj files.")
    #         if entry.name == "L0.obj":
    #             self._robot_link_nodes['L0'].visual = MeshObject(MeshSpecs(file_path=entry))
    #             # print(f"Scene: L0 visual set with file_path: {entry}")
    #         if entry.name == "L1.obj":
    #             self._robot_link_nodes['L1'].visual = MeshObject(MeshSpecs(file_path=entry))
    #         if entry.name == "L2.obj":
    #             self._robot_link_nodes['L2'].visual = MeshObject(MeshSpecs(file_path=entry))
    #         if entry.name == "L3.obj":
    #             self._robot_link_nodes['L3'].visual = MeshObject(MeshSpecs(file_path=entry))
    #         if entry.name == "L4.obj":
    #             self._robot_link_nodes['L4'].visual = MeshObject(MeshSpecs(file_path=entry))
    #         if entry.name == "L5.obj":
    #             self._robot_link_nodes['L5'].visual = MeshObject(MeshSpecs(file_path=entry))
    #         if entry.name == "L6.obj":
    #             self._robot_link_nodes['L6'].visual = MeshObject(MeshSpecs(file_path=entry))
    #     return

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
