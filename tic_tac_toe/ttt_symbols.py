import numpy as np


class TttX:
    """ X-symbol on XY-plane
    """
    def __init__(self, x, y, z, symbol_height_mm: float = 30., rotation_deg: float =  0):
        self._x = x
        self._y = y
        self._z = z
        self._height_mm = symbol_height_mm
        self._rotation_deg = rotation_deg
        self._path = None

        self.generate_path()

    def generate_path(self):
        self._path = np.zeros((8, 7), dtype=np.float32)


