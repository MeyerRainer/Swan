"""
Class for terminal widget.
Author: Rainer Meyer, rot.meyer494@gmail.com
"""

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QGroupBox, QGridLayout

class ProgramWidget(QWidget):

    message_signal = pyqtSignal(str)

    def __init__(self):

        super().__init__()

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
        pass
