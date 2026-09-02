""" MainWindow for Swan app. High level assembly of GUI.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from qt_gui.docks.panels.toolbar import MainToolbar
from qt_gui.docks.dock import *
from qt_gui.viewport.opengl.opengl_viewport import OpenGLViewport
from qt_gui.docks.panels.control_widget import ControlWidget
from qt_gui.docks.panels.gizmo_widget import GizmoWidget
from qt_gui.docks.panels.scene_widget import SceneWidget
from qt_gui.docks.panels.dro_widget import DROWidget
from qt_gui.docks.panels.program_widget import ProgramWidget
from qt_gui.docks.panels.teach_widget import TeachWidget
from qt_gui.docks.panels.vision_widget import VisionWidget
from qt_gui.viewport.camera_widget import CameraWidget

from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import QMainWindow, QApplication
from PyQt6.QtCore import Qt, QSettings, QDir, pyqtSlot
from PyQt6.QtWidgets import QTabWidget


class MainWindow(QMainWindow):

    def __init__(self):

        super().__init__()

        self.settings = QSettings("Meyer", "Swan")

        # Toolbar
        self.toolbar = MainToolbar()
        self.toolbar.setObjectName("Toolbar")

        # Docks
        self.terminal_dock = TerminalDock()
        self.terminal_dock.setObjectName("TerminalDock")
        self.left_dock = LeftDock()
        self.left_dock.setObjectName("LeftDock")
        self.right_dock = RightDock()
        self.right_dock.setObjectName("RightDock")

        # Widgets. Left.
        self.control_panel = ControlWidget()
        self.gizmo_panel = GizmoWidget()
        self.program_panel = ProgramWidget(initial_dir=self.settings.value("last_directory", QDir.homePath()))
        self.scene_panel = SceneWidget()
        # Right.
        self.dro_panel = DROWidget()
        self.teach_panel = TeachWidget()
        self.vision_panel = VisionWidget(calibration_image_dir=self.settings.value("camera_calibration_directory", QDir.homePath()))
        self.terminal = self.terminal_dock.widget()

        # Tabify. Left.
        self.left_dock.tabs.addTab(self.control_panel, "Jog")
        self.left_dock.tabs.addTab(self.gizmo_panel, "Gizmo")
        self.left_dock.tabs.addTab(self.program_panel, "Program")
        self.left_dock.tabs.addTab(self.scene_panel, "Scene")
        # Right.
        self.right_dock.tabs.addTab(self.dro_panel, "DRO")
        self.right_dock.tabs.addTab(self.teach_panel, "Teach")
        self.right_dock.tabs.addTab(self.vision_panel, "Vision")

        # Viewport.
        self.view_scene = OpenGLViewport()              # 3D scene.
        self.view_camera = CameraWidget()               # Camera.
        self.viewport_tabs = QTabWidget()
        self.viewport_tabs.addTab(self.view_scene, "Scene")
        self.viewport_tabs.addTab(self.view_camera, "Camera")

        self.setCentralWidget(self.viewport_tabs)

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
        #menubar.setNativeMenuBar(True)  # MachOS

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
        view_menu.addAction(self.terminal_dock.toggleViewAction())
        view_menu.addAction(self.left_dock.toggleViewAction())
        view_menu.addAction(self.right_dock.toggleViewAction())
        help_menu = menubar.addMenu("Help")

    def build_ui(self):
        self.setWindowTitle("Swan")
        self.setWindowIcon(QIcon("qt_gui/resources/icons/icon256_nobg.png"))
        self.create_menu()
        self.addToolBar(self.toolbar)

        # Remap corners for side docks to stretch from top to bottom
        self.setCorner(Qt.Corner.TopLeftCorner, Qt.DockWidgetArea.LeftDockWidgetArea)
        self.setCorner(Qt.Corner.BottomLeftCorner, Qt.DockWidgetArea.LeftDockWidgetArea)
        self.setCorner(Qt.Corner.TopRightCorner, Qt.DockWidgetArea.RightDockWidgetArea)
        self.setCorner(Qt.Corner.BottomRightCorner, Qt.DockWidgetArea.RightDockWidgetArea)

        # Add docks
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.left_dock)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.right_dock)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.terminal_dock)

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

    def update_status(self, status_dict: dict):
        self.toolbar.update_status(status_dict)
        self.dro_panel.update_status(status_dict)
        # self.view_3d.update_status(status_dict)
        self.control_panel.update_status(status_dict)
