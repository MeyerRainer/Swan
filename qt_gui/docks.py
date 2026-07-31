""" Docks for the Swan app

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtWidgets import QTabWidget

from qt_gui.dock import Dock
from qt_gui.widgets.terminal_widget import TerminalWidget


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
