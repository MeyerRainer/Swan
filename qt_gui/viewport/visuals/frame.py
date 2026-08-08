from qt_gui.viewport.scene import SceneNode
from qt_gui.viewport.visuals.visual import Visual, Material
from qt_gui.viewport.shapes import *

import numpy as np


class ArrowFrame(SceneNode):

    def __init__(self):

        super().__init__()

        self.visuals = [DragArrow(ArrowSpecs(axis='x', colors=(1., 0.2, 0.2, 1.))),
                        DragArrow(ArrowSpecs(axis='y', colors=(0.2, 1., 0.2, 1.))),
                        DragArrow(ArrowSpecs(axis='z', colors=(0.2, 0.2, 1., 1.)))]

        ...

class RingFrame(SceneNode):
    ...

class PlaneFrame(SceneNode):
    ...

class FrameLine(SceneNode):
    ...