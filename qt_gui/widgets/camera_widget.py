"""
Camera widget for the viewport dock
Author: Rainer Meyer, rot.meyer494@gmail.com
"""
from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import Qt

import numpy as np


class CameraWidget(QWidget):

    def __init__(self, parent=None):

        super().__init__(parent)

        self.label = QLabel()
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

        self.current_pixmap = None

    def show_frame(self, frame: np.ndarray):

        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        h, w, ch = frame.shape

        image = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)

        pixmap = QPixmap.fromImage(image)

        self.label.setPixmap(pixmap.scaled(self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.current_pixmap = pixmap

    def resizeEvent(self, event):

        if self.current_pixmap:
            self.label.setPixmap(self.current_pixmap.scaled(self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))