"""
Class for terminal widget.
Author: Rainer Meyer, rot.meyer494@gmail.com
"""

from PyQt6.QtCore import pyqtSignal, QSettings, QDir
from PyQt6.QtGui import QTextCursor, QColor, QTextFormat, QTextCharFormat
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QGroupBox, QGridLayout, QFileDialog, \
    QMessageBox, QPlainTextEdit, QLineEdit, QTextEdit, QLabel
import os


class ProgramEditor(QPlainTextEdit):

    def __init__(self):

        super().__init__()

    def highlight_row(self, line_number: int):
        """Highlights a specific row index (0-based) across its full width."""
        block = self.document().findBlockByLineNumber(line_number)
        if not block.isValid():
            return

        # 1. Instantiate an ExtraSelection container object
        extra_selection = QTextEdit.ExtraSelection()

        # 2. Configure the line format
        fmt = QTextCharFormat()
        fmt.setBackground(QColor("#FFE4B5"))  # Light Moccasin
        fmt.setProperty(QTextFormat.Property.FullWidthSelection, True)
        extra_selection.format = fmt

        # 3. Create a cursor targeted at the specific block
        cursor = QTextCursor(block)
        extra_selection.cursor = cursor

        # 4. Pass a list containing the selection to the widget
        self.setExtraSelections([extra_selection])


class ProgramWidget(QWidget):

    write_terminal = pyqtSignal(str)
    directory_changed = pyqtSignal(str)
    sgn_open_file = pyqtSignal(str, str)

    def __init__(self, initial_dir=None):

        super().__init__()

        self.allowed_extensions = {".swn", ".py"}
        self.current_dir: str = initial_dir or QDir.homePath()

        # Buttons
        self.new_program_button = QPushButton("New program")
        self.load_from_file_button = QPushButton("Load from file")
        self.write_to_file_button = QPushButton("Write to file")
        self.upload_program_button = QPushButton("Upload")
        self.run_button = QPushButton("Run")
        self.pause_button = QPushButton("Pause")
        self.stop_button = QPushButton("Stop")

        # Text box
        # self.text_box = QPlainTextEdit()
        self.text_box = ProgramEditor()
        self.current_line = None  # Line in text box
        self.mdi = QLineEdit()  # Manual data input

        # Labels
        self.current_program_label = QLabel("")

        self.init_ui()
        self.connect_ui()


    def init_ui(self):

        layout = QVBoxLayout()

        group_program_control = QGroupBox("Program control")
        layout_program_control = QVBoxLayout()

        # Button grid
        g_button_layout = QGridLayout()
        g_button_layout.addWidget(self.new_program_button, 0, 0)
        g_button_layout.addWidget(self.load_from_file_button, 0, 1)
        g_button_layout.addWidget(self.write_to_file_button, 0, 2)
        g_button_layout.addWidget(self.upload_program_button, 0, 3)
        g_button_layout.addWidget(self.run_button, 1, 0)
        g_button_layout.addWidget(self.pause_button, 1, 1)
        g_button_layout.addWidget(self.stop_button, 1, 2)
        layout_program_control.addLayout(g_button_layout)

        # Current file
        h_layout_file = QHBoxLayout()
        h_layout_file.addWidget(QLabel("Program: "))
        h_layout_file.addWidget(self.current_program_label)
        layout_program_control.addLayout(h_layout_file)

        # Text box
        v_text_box_layout = QVBoxLayout()
        v_text_box_layout.addWidget(self.text_box)
        v_text_box_layout.addWidget(self.mdi)
        layout_program_control.addLayout(v_text_box_layout)

        group_program_control.setLayout(layout_program_control)
        layout.addWidget(group_program_control)

        self.setLayout(layout)

    def connect_ui(self):
        self.load_from_file_button.clicked.connect(self.open_file_dialog)

    def open_file_dialog(self):
        # last_dir: str = self.settings.value("last_directory", QDir.homePath())

        file_path, _ = QFileDialog.getOpenFileName(self, "Select a File", self.current_dir,"All Files (*);;Text Files (*.txt);;Python Files (*.py)",)

        if not file_path:
            return

        # Update last directory
        self.current_dir = os.path.dirname(file_path)
        self.directory_changed.emit(self.current_dir)

        file, ext = os.path.splitext(file_path)
        ext = ext.lower()
        if ext not in self.allowed_extensions:
            QMessageBox.warning(self, "Invalid File Type", f"Unsupported file format '{ext}'.\nPlease select a .txt, .json, or .csv file.")
            return

        self.sgn_open_file.emit(file, ext)

    def on_lineno_change(self, lineno: int):
        """ Highlights the instruction in execution.
        :param lineno: Line number of the instruction.
        """
        self.text_box.highlight_row(line_number=lineno)

    def on_file_load(self, file_name: str, file_contents: str):
        self.current_program_label.setText(file_name)
        self.text_box.setPlainText(file_contents)
        self.write_terminal.emit("File loaded.")