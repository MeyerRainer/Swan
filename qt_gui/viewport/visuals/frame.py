""" Renderable frames.

Author: Rainer Meyer, r.meyer494@gmail.com
"""


from qt_gui.viewport.render.renderer import RenderContext
from qt_gui.viewport.visuals.visual import Renderable
from qt_gui.viewport.shapes import *
from robot_math.pose import Pose


# Triangle rendered arrow frame
@dataclass
class ArrowFrame(Renderable):
    x: DragArrow = DragArrow(ArrowSpecs(axis=(1., 0., 0.), colors=(1., 0.2, 0.2, 1.)))
    y: DragArrow = DragArrow(ArrowSpecs(axis=(0., 1., 0.), colors=(0.2, 1., 0.2, 1.)))
    z: DragArrow = DragArrow(ArrowSpecs(axis=(0., 0., 1.), colors=(0.2, 0.2, 1., 1.)))
    pose: Pose = Pose.identity()

    def render(self, context: RenderContext, pose: Pose) -> None:
        for arrow in [self.x, self.y, self.z]:
            context.renderer.render_visual(arrow, pose)

@dataclass
class RingFrame(Renderable):
    yz: RotationRing = RotationRing(RingSpecs(normal_axis=(1., 0., 0.), colors=(1., 0.2, 0.2, 0.7)))
    zx: RotationRing = RotationRing(RingSpecs(normal_axis=(0., 1., 0.), colors=(0.2, 1., 0.2, 0.7)))
    xy: RotationRing = RotationRing(RingSpecs(normal_axis=(0., 0., 1.), colors=(0.2, 0.2, 1., 0.7)))
    pose: Pose = Pose.identity()

    def render(self, context: RenderContext, pose: Pose):
        for ring in [self.yz, self.zx, self.xy]:
            context.renderer.render_visual(ring, pose)

class PlaneFrame(Renderable):
    yz: DragPlane = DragPlane(PlaneSpecs(u_dir=(0., 1., 0.), v_dir=(0., 0., 1.), colors=(1., 0.2, 0.2, 0.7)))
    zx: DragPlane = DragPlane(PlaneSpecs(u_dir=(0., 0., 1.), v_dir=(1., 0., 0.), colors=(0.2, 1., 0.2, 0.7)))
    xy: DragPlane = DragPlane(PlaneSpecs(u_dir=(1., 0., 0.), v_dir=(0., 1., 0.), colors=(0.2, 0.2, 1., 0.7)))
    pose: Pose = Pose.identity()

    def render(self, context: RenderContext, pose: Pose):
        for plane in [self.yz, self.zx, self.xy]:
            context.renderer.render_visual(plane, pose)

# # Line-rendered frame
# class LineFrame(SceneNode):
#     ...