"""
Class for toolbar
Author: Rainer Meyer, rot.meyer494@gmail.com
"""

from PyQt6.QtWidgets import QToolBar, QPushButton, QComboBox, QLabel
from PyQt6.QtCore import Qt


class MainToolbar(QToolBar):

    def __init__(self):

        super().__init__()

        self.connect_button = QPushButton("Connect")
        self.refresh_button = QPushButton("Refresh")
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems([
                "9600",
                "115200",
                "230400"
        ])
        self.baud_combo.setCurrentIndex(1)

        self.grbl_status_label = QLabel('')
        self.grbl_status_label.setMinimumWidth(40)
        self.grbl_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._build_toolbar()

    def _build_toolbar(self):
        self.connect_button.setCheckable(True)  # Toggleable?
        self.addWidget(self.connect_button)
        self.connect_button.setStyleSheet("background-color: red")
        self.addWidget(self.refresh_button)
        self.addSeparator()
        self.addWidget(self.port_combo)
        self.addWidget(self.baud_combo)
        self.addSeparator()
        self.addWidget(QLabel("GRBL: "))
        self.addWidget(self.grbl_status_label)
        self.addSeparator()

    def on_connect(self):
        self.connect_button.setText("Disconnect")
        self.connect_button.setStyleSheet("background-color: green")

    def on_disconnect(self):
        self.connect_button.setText("Connect")
        self.connect_button.setStyleSheet("background-color: red")

    def update_status(self, status: str):
        if status == "Idle":
            self.grbl_status_label.setStyleSheet("background-color: green")
        elif status == "Run":
            self.grbl_status_label.setStyleSheet("background-color: red")

        self.grbl_status_label.setText(status)
