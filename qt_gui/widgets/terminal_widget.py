""" Class for terminal widget.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QPlainTextEdit, QLineEdit

class TerminalWidget(QWidget):

    command_signal = pyqtSignal(str)

    def __init__(self):

        super().__init__()

        self.output = QPlainTextEdit()
        self.command = QLineEdit()
        self.send_button = QPushButton("Send")
        self.clear_button = QPushButton("Clear")

        self.init_ui()
        self.connect_ui()


    def init_ui(self):
        layout = QVBoxLayout()

        self.output.setReadOnly(True)

        lower = QHBoxLayout()

        lower.addWidget(self.clear_button)
        lower.addWidget(self.command)
        lower.addWidget(self.send_button)

        layout.addWidget(self.output)
        layout.addLayout(lower)

        self.setLayout(layout)

    def connect_ui(self):
        self.send_button.clicked.connect(self.send_command)
        self.command.returnPressed.connect(self.send_command)
        self.clear_button.clicked.connect(self.clear)

    def send_command(self):
        msg = self.command.text()
        self.command_signal.emit(msg)
        self.command.clear()

    def write(self,text):
        self.output.appendPlainText(text)

    def clear(self):
        self.output.clear()
