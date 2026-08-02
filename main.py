"""
Swan is a robot manipulator controller application communicating with GRBL.
Author: Rainer Meyer, r.meyer494@gmail.com
"""


from main_window import MainWindow
from application_context import ApplicationContext
from application_controller import ApplicationController
from PyQt6.QtWidgets import QApplication
import sys
import ctypes

class Application:

    def __init__(self):

        self.app_id = "rainer.swan.0.1"
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(self.app_id)

        self.app = QApplication(sys.argv)

        self.window = MainWindow()
        self.context = ApplicationContext()
        self.controller = ApplicationController(self.window, self.context)

    def run(self):
        self.window.show()
        sys.exit(self.app.exec())

def main():

    swan = Application()

    swan.run()


if __name__ == "__main__":
    main()
