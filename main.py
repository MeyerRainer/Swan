"""
Swan is a robot manipulator controller application communicating with GRBL.
Author: Rainer Meyer, r.meyer494@gmail.com
"""


from qt_gui.main_window import MainWindow
from PyQt6.QtWidgets import QApplication
import sys
import ctypes

def main():
    app_id = "rainer.swan.subproduct.version"
    ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(app_id)
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
