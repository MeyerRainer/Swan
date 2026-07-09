"""
Class for terminal widget.
Author: Rainer Meyer, rot.meyer494@gmail.com
"""

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QGroupBox

class TeachWidget(QWidget):

    message_signal = pyqtSignal(str)

    def __init__(self):

        super().__init__()

        self.save_point_button = QPushButton("Save point")

        self.init_ui()
        self.connect_ui()


    def init_ui(self):
        layout =  QVBoxLayout()

        group_teach = QGroupBox("Teaching")
        layout_teach = QVBoxLayout()

        lower = QHBoxLayout()

        lower.addWidget(self.save_point_button)

        layout_teach.addLayout(lower)
        group_teach.setLayout(layout_teach)

        layout.addWidget(group_teach)

        self.setLayout(layout)

    def connect_ui(self):
        pass
