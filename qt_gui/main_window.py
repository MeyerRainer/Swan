""" MainWindow for Swan app. High level assembly of GUI.
Author: Rainer Meyer, rot.meyer494@gmail.com
"""
from PyQt6.QtGui import QIcon

from backend import application_interface
from qt_gui.toolbar import MainToolbar
from qt_gui.docks import *
from qt_gui.viewport.view_3d import View3D
from qt_gui.viewport.camera_widget import CameraWidget

from PyQt6.QtWidgets import QMainWindow, QApplication
from PyQt6.QtCore import Qt, QSettings, QDir, pyqtSlot
from PyQt6.QtWidgets import QTabWidget


class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        self.settings = QSettings("Meyer", "Swan")



        # Toolbar
        self.toolbar = MainToolbar()

        # Docks
        self.control = ControlDock()
        self.dro = DRODock()
        self.terminal = TerminalDock()
        self.teach_interface = TeachDock()
        self.program_control = ProgramDock(initial_dir=self.settings.value("last_directory", QDir.homePath()))

        # Viewport
        self.view_3d = View3D()
        self.view_camera = CameraWidget()
        self.viewport_tabs = QTabWidget()
        self.viewport_tabs.addTab(self.view_3d, "3D")
        self.viewport_tabs.addTab(self.view_camera, "Camera")

        self.setCentralWidget(self.viewport_tabs)

        # Application interface
        self.api = application_interface.ApplicationInterface(self)

        # Allow for nested docking
        self.setDockNestingEnabled(True)

        # Dock settings
        self.setDockOptions(
            QMainWindow.DockOption.AllowNestedDocks |
            QMainWindow.DockOption.AllowTabbedDocks |
            QMainWindow.DockOption.AnimatedDocks
        )

        self.restore_previous_layout()

        self.build_ui()

    def create_menu(self):
        menubar = self.menuBar()

        # Menus
        file_menu = menubar.addMenu("File")
        file_menu.addAction("Open")
        file_menu.addAction("Save")
        file_menu.addSeparator()
        # Options
        options_menu = menubar.addMenu("Options")
        options_menu.addAction("Quit", QApplication.quit)
        # Tools
        tools_menu = menubar.addMenu("Tools")
        tools_menu.addAction("Save layout", self.save_current_layout)
        tools_menu.addAction("Restore default layout", self.restore_default_layout)
        # View
        view_menu = menubar.addMenu("View")
        view_menu.addAction(self.control.toggleViewAction())
        view_menu.addAction(self.dro.toggleViewAction())
        view_menu.addAction(self.terminal.toggleViewAction())
        view_menu.addAction(self.teach_interface.toggleViewAction())
        view_menu.addAction(self.program_control.toggleViewAction())
        help_menu = menubar.addMenu("Help")

    def build_ui(self):
        self.setWindowTitle("Swan")
        self.setWindowIcon(QIcon("resources/icons/icon256_nobg.png"))
        self.create_menu()
        self.addToolBar(self.toolbar)

        # Remap corners for side docks to stretch from top to bottom
        self.setCorner(Qt.Corner.TopLeftCorner, Qt.DockWidgetArea.LeftDockWidgetArea)
        self.setCorner(Qt.Corner.BottomLeftCorner, Qt.DockWidgetArea.LeftDockWidgetArea)
        self.setCorner(Qt.Corner.TopRightCorner, Qt.DockWidgetArea.RightDockWidgetArea)
        self.setCorner(Qt.Corner.BottomRightCorner, Qt.DockWidgetArea.RightDockWidgetArea)

        # Add docks
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.control)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.program_control)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.dro)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.teach_interface)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.terminal)

        # Tabify
        self.tabifyDockWidget(self.program_control, self.control)
        self.tabifyDockWidget(self.teach_interface, self.dro)

        # Set viewport as central widget
        self.setCentralWidget(self.viewport_tabs)

    def save_current_layout(self):
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("windowState", self.saveState())
        self.settings.setValue("size", self.size())

    def restore_previous_layout(self):
        geometry = self.settings.value("geometry")
        if geometry:
            self.restoreGeometry(geometry)
        state = self.settings.value("WindowState")
        if state:
            self.restoreState(state)
        size = self.settings.value("size")
        if size:
            self.resize(size)

    def restore_default_layout(self):
        # Clear specific keys from registry/config file
        self.settings.remove("geometry")
        self.settings.remove("windowState")

        # Reset window size and position hardcoded defaults
        self.resize(1400, 900)

    @pyqtSlot(str)
    def save_last_directory(self, new_dir: str):
        self.settings.setValue("last_directory", new_dir)

    def closeEvent(self, event):
        """ Automatically saves layout right before application exit."""
        self.save_current_layout()
        event.accept()
