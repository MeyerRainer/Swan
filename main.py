"""
Swan is a robot controller application communicating with GRBL.
Author: Rainer Meyer, rot.meyer494@gmail.com
"""


from qt_gui.main_window import MainWindow
from PyQt6.QtWidgets import QApplication
import sys

def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
