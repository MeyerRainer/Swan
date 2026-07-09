"""
Docks for the Swan app
Author: Rainer Meyer, rot.meyer494@gmail.com
"""

from qt_gui.dock import Dock
from qt_gui.widgets.control_widget import ControlWidget
from qt_gui.widgets.dro_widget import DROWidget
from qt_gui.widgets.terminal_widget import TerminalWidget
from qt_gui.widgets.teach_widget import TeachWidget
from qt_gui.widgets.program_widget import ProgramWidget


class ControlDock(Dock):
    def __init__(self):

        super().__init__("Control", ControlWidget())


class DRODock(Dock):
    def __init__(self):

        super().__init__("DRO", DROWidget())


class TerminalDock(Dock):
    def __init__(self):
        super().__init__("terminal", TerminalWidget())


class TeachDock(Dock):
    def __init__(self):
        super().__init__("teach", TeachWidget())


class ProgramDock(Dock):
    def __init__(self):
        super().__init__("program", ProgramWidget())
