""" Class for scene widget.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtWidgets import *
from qt_gui.viewport.scene.scene import *


class SceneWidget(QWidget):

    def __init__(self):

        super().__init__()

        self.scene_tree = QTreeView()

        # Setup scene tree.
        self.expand_collapse_tree = QPushButton("Expand All")
        self.scene_tree.setDragEnabled(True)
        self.scene_tree.setAcceptDrops(True)
        self.scene_tree.setDropIndicatorShown(True)
        self.scene_tree.setDefaultDropAction(Qt.DropAction.MoveAction)
        self.scene_tree.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)

        self.x_input = QLineEdit()
        self.y_input = QLineEdit()
        self.z_input = QLineEdit()
        self.rz1_input = QLineEdit()
        self.ry_input = QLineEdit()
        self.rz2_input = QLineEdit()

        self.init_ui()
        self.connect_ui()

    def init_ui(self):
        scene_panel_layout = QVBoxLayout()

        # ======================================= Top Button Row ========================================
        h_button_layout = QHBoxLayout()
        h_button_layout.addWidget(self.expand_collapse_tree)

        scene_panel_layout.addLayout(h_button_layout)

        # ========================================= Scene Tree ==========================================
        self.scene_tree.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)  # Read-only text
        self.scene_tree.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        scene_panel_layout.addWidget(self.scene_tree)

        # ========================================= Modify box ==========================================
        group_box_modify = QGroupBox("Modify")
        v_layout_modify = QVBoxLayout()

        h_pose_input = QHBoxLayout()
        h_pose_input.addWidget(QLabel("X"))
        h_pose_input.addWidget(self.x_input)
        h_pose_input.addWidget(QLabel("Y"))
        h_pose_input.addWidget(self.y_input)
        h_pose_input.addWidget(QLabel("Z"))
        h_pose_input.addWidget(self.z_input)
        h_pose_input.addWidget(QLabel("RZ"))
        h_pose_input.addWidget(self.rz1_input)
        h_pose_input.addWidget(QLabel("RY"))
        h_pose_input.addWidget(self.ry_input)
        h_pose_input.addWidget(QLabel("RZ"))
        h_pose_input.addWidget(self.rz2_input)
        v_layout_modify.addLayout(h_pose_input)

        group_box_modify.setLayout(v_layout_modify)
        scene_panel_layout.addWidget(group_box_modify)


        self.setLayout(scene_panel_layout)

    def connect_ui(self):
        self.expand_collapse_tree.clicked.connect(self.on_expand_collapse_button)

    def on_expand_collapse_button(self):
        if self.expand_collapse_tree.text() == "Expand All":
            self.on_expand_tree()
        else:
            self.on_collapse_tree()

    def on_collapse_tree(self):
        self.expand_collapse_tree.setText("Expand All")
        self.scene_tree.collapseAll()

    def on_expand_tree(self):
        self.expand_collapse_tree.setText("Collapse All")
        self.scene_tree.expandAll()

