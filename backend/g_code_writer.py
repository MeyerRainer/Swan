import math
from datetime import datetime


class GCodeWriter:
    def __init__(self, f_name: str=None):
        self._file_open = False
        self._file_path = f_name

        # Open file
        if self._file_path is not None:
            self._file_open = True
            self.file = open(self._file_path, 'w')
            self.write_comment("Generated at " + datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

        self.decimals = 2

    # def __enter__(self):
    #     self.file = open(self.filepath, 'w')
    #     self.write_comment("G-code file created.")
    #     return self

    # def __exit__(self, exc_type, exc_val, exc_tb):
    #     self.write_comment("End of G-code file.")
    #     self.file.close()

    def file_is_open(self) -> bool:
        return self._file_open

    def write_line(self, line: str):
        if self._file_path is None:
            raise ValueError("No file path  given!")
        self.file.write(line.strip() + '\n')

    def write_comment(self, comment: str):
        if self._file_path is None:
            raise ValueError("No file path  given!")
        self.write_line(f"( {comment} )")

    def write_raw(self, gcode: str):
        self.write_line(gcode)

    def move_linear(self, x=None, y=None, z=None, a=None, b=None, c=None, u=None, v=None, feedrate=None) -> str:
        """Write a linear move (G1) command with optional coordinates and feedrate."""
        parts = ["G1"]
        if x is not None:
            parts.append(f"X{x:.{self.decimals}f}")
        if y is not None:
            parts.append(f"Y{y:.{self.decimals}f}")
        if z is not None:
            parts.append(f"Z{z:.{self.decimals}f}")
        if a is not None:
            parts.append(f"A{a:.{self.decimals}f}")
        if b is not None:
            parts.append(f"B{b:.{self.decimals}f}")
        if c is not None:
            parts.append(f"C{c:.{self.decimals}f}")
        if u is not None:
            parts.append(f"U{u:.{self.decimals}f}")
        if v is not None:
            parts.append(f"V{v:.{self.decimals}f}")

        if feedrate is not None:
            parts.append(f"F{int(math.ceil(feedrate))}")

        return "".join(parts)

    def move_rapid(self, x=None, y=None, z=None) -> str:
        """Write a rapid move (G0) command."""
        parts = ["G0"]
        if x is not None:
            parts.append(f"X{x:.{self.decimals}f}")
        if y is not None:
            parts.append(f"Y{y:.{self.decimals}f}")
        if z is not None:
            parts.append(f"Z{z:.{self.decimals}f}")
        return " ".join(parts)
