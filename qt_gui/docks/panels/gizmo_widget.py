""" Class for gizmo control widget.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtCore import pyqtSignal, QDir
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QGroupBox, QComboBox

import config


class GizmoWidget(QWidget):

    sgn_write_terminal = pyqtSignal(str)

    def __init__(self, calibration_image_dir: str = None):

        super().__init__()

        self.execute_planned_button = QPushButton("Execute Planned")
        self.revert_planned_button = QPushButton("Revert Planned")

        self.frame_select = QComboBox()
        self.frame_select.addItems(config.FRAMES)

        self.init_ui()
        self.connect_ui()

        self.current_dir: str = calibration_image_dir or QDir.homePath()


    def init_ui(self):
        layout =  QVBoxLayout()

        group_planning = QGroupBox("Planning")
        v_box_planning = QVBoxLayout()
        h_planning = QHBoxLayout()
        h_planning.addWidget(self.execute_planned_button)
        h_planning.addWidget(self.revert_planned_button)
        v_box_planning.addLayout(h_planning)
        group_planning.setLayout(v_box_planning)
        layout.addWidget(group_planning)

        group_gizmo = QGroupBox("Gizmo")
        v_gizmo = QVBoxLayout()
        v_gizmo.addWidget(self.frame_select)
        group_gizmo.setLayout(v_gizmo)
        layout.addWidget(group_gizmo)

        self.setLayout(layout)

    def connect_ui(self): ...
