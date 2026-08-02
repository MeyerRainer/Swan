"""
Class for scene widget.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

from PyQt6.QtWidgets import *
from PyQt6.QtCore import Qt
from qt_gui.viewport.scene import *

class SceneWidget(QWidget):

    def __init__(self):

        super().__init__()

        self.scene_tree = QTreeView()

        self.collapse_all_button = QPushButton("Collapse ALl")
        self.expand_all_button = QPushButton("Expand All")

        self.layout = QVBoxLayout()

        self.setLayout(self.layout)

        self.init_ui()
        self.connect_ui()

    def init_ui(self):

        self.scene_tree.expandAll()
        self.scene_tree.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)  # Read-only text
        self.scene_tree.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)

        self.layout.addWidget(self.scene_tree)

    def connect_ui(self):
        self.collapse_all_button.clicked.connect(lambda: self.scene_tree.collapseAll)
        self.expand_all_button.clicked.connect(lambda: self.scene_tree.expandAll)