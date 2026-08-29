""" Camera widget for the viewport dock for rendering camera / vision frames.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout, QSizePolicy
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import Qt

import numpy as np


class CameraWidget(QWidget):

    def __init__(self, parent=None):

        super().__init__(parent)

        self.label = QLabel()
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # Allow the label to shrink below the pixmap's actual size
        self.label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.label)

        self.current_pixmap = None

    def show_frame(self, frame: np.ndarray):
        dimensions = frame.shape
        height, width = dimensions[0], dimensions[1]
        # if len(dimensions) < 3:
        #     channels = 1
        # else:
        #     channels = dimensions[2]
        assert frame.shape[2] == 3

        bytes_per_line = frame.strides[0]
        image = QImage(frame.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()

        self.current_pixmap = QPixmap.fromImage(image)
        self._update_pixmap()


    def resizeEvent(self, event):

        super().resizeEvent(event)

        self._update_pixmap()

    def _update_pixmap(self):
        if self.current_pixmap and not self.label.size().isEmpty():
            self.label.setPixmap(self.current_pixmap.scaled(self.label.size(),Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))



# class CameraWidget(QWidget):
#
#     def __init__(self, parent=None):
#
#         super().__init__(parent)
#
#         self.label = QLabel()
#         self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
#
#         layout = QVBoxLayout(self)
#         layout.addWidget(self.label)
#
#         self.current_pixmap = None
#
#     def show_frame(self, frame: np.ndarray):
#         # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#
#         h, w, ch = frame.shape
#
#         image = QImage(frame.data, w, h, ch * w, QImage.Format.Format_RGB888)
#
#         pixmap = QPixmap.fromImage(image)
#
#         self.label.setPixmap(pixmap.scaled(self.label.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
#         self.current_pixmap = pixmap
#
#     def resizeEvent(self, event):
#
#         if self.current_pixmap:
#             self.label.setPixmap(self.current_pixmap.scaled(self.label.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))