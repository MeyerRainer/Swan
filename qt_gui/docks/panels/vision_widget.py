""" Class for vision widget.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
import os

from PyQt6.QtCore import pyqtSignal, QDir
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QGroupBox, QFileDialog, QMessageBox


class VisionWidget(QWidget):

    sgn_write_terminal = pyqtSignal(str)
    sgn_open_file = pyqtSignal()
    sgn_camera_calibration = pyqtSignal()
    sgn_find_camera_pose = pyqtSignal()
    sgn_game_reset = pyqtSignal()
    sgn_execute_machine_move = pyqtSignal()
    # sgn_directory_changed = pyqtSignal(str)
    # sgn_open_file = pyqtSignal(str)

    def __init__(self, calibration_image_dir: str = None):

        super().__init__()

        self.allowed_extensions = ["jpg, png"]
        self.load_images_button = QPushButton("Load Images")
        self.calibrate_camera_button = QPushButton("Calibrate Camera")
        self.locate_camera_button = QPushButton("Locate Camera")
        self.reset_game_button = QPushButton("Reset Game")
        self.machine_move_button = QPushButton("Machine Execute")

        self.init_ui()
        self.connect_ui()

        self.current_dir: str = calibration_image_dir or QDir.homePath()


    def init_ui(self):
        layout =  QVBoxLayout()

        group_camera_calibration = QGroupBox("Camera Calibration")
        v_box_calibration = QVBoxLayout()
        h_calibration = QHBoxLayout()
        h_calibration.addWidget(self.load_images_button)
        h_calibration.addWidget(self.calibrate_camera_button)
        h_calibration.addWidget(self.locate_camera_button)
        v_box_calibration.addLayout(h_calibration)
        group_camera_calibration.setLayout(v_box_calibration)
        layout.addWidget(group_camera_calibration)

        group_ttt_game = QGroupBox("Tic Tac Toe")
        v_box_game = QVBoxLayout()
        v_box_game.addWidget(self.reset_game_button)
        v_box_game.addWidget(self.machine_move_button)
        group_ttt_game.setLayout(v_box_game)
        layout.addWidget(group_ttt_game)

        self.setLayout(layout)

    def connect_ui(self):
        # Camera calibration.
        self.load_images_button.clicked.connect(self.sgn_write_terminal.emit)
        self.calibrate_camera_button.clicked.connect(self.sgn_camera_calibration.emit)
        self.locate_camera_button.clicked.connect(self.sgn_find_camera_pose.emit)

        # Tic-tac-toe Game.
        self.reset_game_button.clicked.connect(self.sgn_game_reset.emit)
        self.machine_move_button.clicked.connect(self.sgn_execute_machine_move)


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