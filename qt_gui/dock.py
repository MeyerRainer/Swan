"""
Dock-class for the Swan app.
Author: Rainer Meyer, rot.meyer494@gmail.com
"""


from PyQt6.QtWidgets import QDockWidget
from PyQt6.QtCore import Qt

class Dock(QDockWidget):
    def __init__(self, title, widget=None, parent=None):
        super().__init__(title, parent)

        # self.setAllowedAreas(
        #     Qt.DockWidgetArea.LeftDockWidgetArea |
        #     Qt.DockWidgetArea.RightDockWidgetArea |
        #     Qt.DockWidgetArea.BottomDockWidgetArea |
        #     Qt.DockWidgetArea.TopDockWidgetArea
        # )
        #
        # self.setFeatures(
        #     QDockWidget.DockWidgetClosable |
        #     QDockWidget.DockWidgetMovable |
        #     QDockWidget.DockWidgetFloatable
        # )
        # 1. Allowed Areas
        self.setAllowedAreas(
            Qt.DockWidgetArea.LeftDockWidgetArea |
            Qt.DockWidgetArea.RightDockWidgetArea |
            Qt.DockWidgetArea.BottomDockWidgetArea |
            Qt.DockWidgetArea.TopDockWidgetArea
        )

        # 2. Features
        self.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetClosable |
            QDockWidget.DockWidgetFeature.DockWidgetMovable |
            QDockWidget.DockWidgetFeature.DockWidgetFloatable
        )

        if widget:
            self.setWidget(widget)
