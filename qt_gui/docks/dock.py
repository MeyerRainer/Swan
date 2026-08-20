""" Docks for the Swan app.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtWidgets import QDockWidget
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QTabWidget
from qt_gui.docks.panels.terminal_widget import TerminalWidget


# Custom Dock
class Dock(QDockWidget):
    def __init__(self, title=None, widget=None, parent=None):
        super().__init__(title, parent)

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


# App docks.
class TerminalDock(Dock):
    def __init__(self):
        super().__init__("terminal", TerminalWidget())


class LeftDock(Dock):

    def __init__(self):

        super().__init__(title="Control")

        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.TabPosition.North)

        self.setWidget(self.tabs)


class RightDock(Dock):

    def __init__(self):

        super().__init__(title="Display")

        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.TabPosition.North)

        self.setWidget(self.tabs)
