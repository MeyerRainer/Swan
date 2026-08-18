""" Class for vision widget.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

from PyQt6.QtCore import pyqtSignal, QDir
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QGroupBox


class VisionWidget(QWidget):

    sgn_write_terminal = pyqtSignal(str)

    def __init__(self, calibration_image_dir: str = None):

        super().__init__()

        self.calibrate_on_images = QPushButton("Load Images")

        self.init_ui()
        self.connect_ui()

        self.current_dir: str = calibration_image_dir or QDir.homePath()


    def init_ui(self):
        layout =  QVBoxLayout()

        group_teach = QGroupBox("Camera Calibration")
        layout_teach = QVBoxLayout()

        lower = QHBoxLayout()
        lower.addWidget(self.calibrate_on_images)

        layout_teach.addLayout(lower)
        group_teach.setLayout(layout_teach)

        layout.addWidget(group_teach)

        self.setLayout(layout)

    def connect_ui(self):
        pass
