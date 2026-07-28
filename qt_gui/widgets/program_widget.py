"""
Class for terminal widget.
Author: Rainer Meyer, rot.meyer494@gmail.com
"""

from PyQt6.QtCore import pyqtSignal, QSettings, QDir
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QGroupBox, QGridLayout, QFileDialog, \
    QMessageBox
import os


class ProgramWidget(QWidget):

    message_signal = pyqtSignal(str)
    directory_changed = pyqtSignal(str)
    file_loaded = pyqtSignal(str, str)

    def __init__(self, initial_dir=None):

        super().__init__()

        self.allowed_extensions = {".swn", ".py"}
        self.current_dir: str = initial_dir or QDir.homePath()

        self.new_program_button = QPushButton("New program")
        self.load_file_button = QPushButton("Load file")
        self.run_button = QPushButton("Run")
        self.pause_button = QPushButton("Pause")
        self.stop_button = QPushButton("Stop")

        self.init_ui()
        self.connect_ui()


    def init_ui(self):

        layout = QVBoxLayout()

        group_program_control = QGroupBox("Program control")
        layout_program_control = QVBoxLayout()


        lower = QHBoxLayout()

        lower.addWidget(self.new_program_button)
        lower.addWidget(self.load_file_button)
        lower.addWidget(self.run_button)
        lower.addWidget(self.pause_button)
        lower.addWidget(self.stop_button)

        layout_program_control.addLayout(lower)
        group_program_control.setLayout(layout_program_control)

        layout.addWidget(group_program_control)

        self.setLayout(layout)

    def connect_ui(self):
        self.load_file_button.clicked.connect(self.open_file_dialog)

    def open_file_dialog(self):
        # last_dir: str = self.settings.value("last_directory", QDir.homePath())

        file_path, _ = QFileDialog.getOpenFileName(self, "Select a File", self.current_dir,"All Files (*);;Text Files (*.txt);;Python Files (*.py)",)

        if not file_path:
            return

        # Update last directory
        self.current_dir = os.path.dirname(file_path)
        self.directory_changed.emit(self.current_dir)

        file, ext = os.path.splitext(file_path)
        ext = ext.lower()
        if ext not in self.allowed_extensions:
            QMessageBox.warning(self, "Invalid File Type", f"Unsupported file format '{ext}'.\nPlease select a .txt, .json, or .csv file.")
            return

        self.file_loaded.emit(file, ext)
