""" Class for toolbar

Author: Rainer Meyer, r.meyer494@gmail.com
"""

from PyQt6.QtWidgets import QToolBar, QPushButton, QComboBox, QLabel
from PyQt6.QtCore import Qt, pyqtSignal


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
        self.baud_combo.setCurrentIndex(1)  # Auto by default.

        self.mcu_status_label = QLabel("Controller: ")
        self.mcu_status_label.setMinimumWidth(120)
        self.mcu_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.hold_resume_button = QPushButton("Feed Hold")

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
        self.addWidget(self.mcu_status_label)
        self.addSeparator()
        self.addWidget(self.hold_resume_button)

    def on_serial_toggle(self, serial_connect: bool):
        if serial_connect:
            self.on_connect()
        else:
            self.disconnect()

    def on_connect(self):
        self.connect_button.setText("Disconnect")
        self.connect_button.setStyleSheet("background-color: green")

    def on_disconnect(self):
        self.connect_button.setText("Connect")
        self.connect_button.setStyleSheet("background-color: red")

    # def update_status(self, status: str):
    #     if status == "Idle":
    #         self.mcu_status_label.setText("Controller: Idle")
    #         self.mcu_status_label.setStyleSheet("background-color: green; border-radius: 5px; padding: 5px;")
    #     elif status == "Run":
    #         self.mcu_status_label.setText("Controller: Run")
    #         self.mcu_status_label.setStyleSheet("background-color: red; border-radius: 5px; padding: 5px;")
    def update_status(self, status_dict: dict):
        status = status_dict["status"]
        match status:
            case "Idle":
                self.mcu_status_label.setText("Controller: Idle")
                self.mcu_status_label.setStyleSheet("background-color: grey; border-radius: 5px; padding: 5px;")
            case "Run":
                self.mcu_status_label.setText("Controller: Run")
                self.mcu_status_label.setStyleSheet("background-color: green; border-radius: 5px; padding: 5px;")
                self.on_cycle_start()
            case "Hold":
                self.mcu_status_label.setText("Controller: Hold")
                self.mcu_status_label.setStyleSheet("background-color: orange; border-radius: 5px; padding: 5px;")
                self.on_feed_hold()
            case "Home":
                self.mcu_status_label.setText("Controller: Homing")
                self.mcu_status_label.setStyleSheet("background-color: blue; border-radius: 5px; padding: 5px;")
            case "Alarm":
                self.mcu_status_label.setText("Controller: Alarm")
                self.mcu_status_label.setStyleSheet("background-color: red; border-radius: 5px; padding: 5px;")
            case "Check":
                self.mcu_status_label.setText("Controller: Check Mode")
                self.mcu_status_label.setStyleSheet("background-color: tan; border-radius: 5px; padding: 5px;")
            case "Door":
                self.mcu_status_label.setText("Controller: Safety Door")
                self.mcu_status_label.setStyleSheet("background-color: red; border-radius: 5px; padding: 5px;")

    def display_ports(self, ports: list[str]):
        self.port_combo.clear()
        self.port_combo.addItems(ports)

    def on_feed_hold(self):
        self.hold_resume_button.setText("Resume")

    def on_cycle_start(self):
        self.hold_resume_button.setText("Pause")
