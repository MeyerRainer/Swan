""" Class for vision widget.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
import os

from PyQt6.QtCore import pyqtSignal, QDir
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QGroupBox, QFileDialog, QMessageBox


class VisionWidget(QWidget):

    sgn_write_terminal = pyqtSignal(str)
    sgn_open_file = pyqtSignal()
    # sgn_directory_changed = pyqtSignal(str)
    # sgn_open_file = pyqtSignal(str)

    def __init__(self, calibration_image_dir: str = None):

        super().__init__()

        self.allowed_extensions = ["jpg, png"]
        self.load_images_button = QPushButton("Load Images")

        self.init_ui()
        self.connect_ui()

        self.current_dir: str = calibration_image_dir or QDir.homePath()


    def init_ui(self):
        layout =  QVBoxLayout()

        group_teach = QGroupBox("Camera Calibration")
        layout_teach = QVBoxLayout()

        lower = QHBoxLayout()
        lower.addWidget(self.load_images_button)

        layout_teach.addLayout(lower)
        group_teach.setLayout(layout_teach)

        layout.addWidget(group_teach)

        self.setLayout(layout)

    def connect_ui(self):
        self.load_images_button.clicked.connect(self.sgn_write_terminal.emit)

    # def open_file_dialog(self):
    #     # last_dir: str = self.settings.value("last_directory", QDir.homePath())
    #
    #     file_path, _ = QFileDialog.getOpenFileName(self, "Select a File", self.current_dir,"All Files (*);;Images (*.png);(*.jpg)")
    #     if not file_path:
    #         return
    #
    #     # Update last directory
    #     self.current_dir = os.path.dirname(file_path)
    #     self.sgn_directory_changed.emit(self.current_dir)
    #
    #     file, extension = os.path.splitext(file_path)
    #     extension = extension.lower()
    #     if extension not in self.allowed_extensions:
    #         QMessageBox.warning(self, "Invalid File Type", f"Unsupported file format '{extension}'")
    #         return
    #
    #     self.sgn_open_file.emit(file, extension)