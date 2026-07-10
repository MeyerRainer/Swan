"""
Class for viewport and its core components
Author: Rainer Meyer, rot.meyer494@gmail.com
"""


#from PyQt5.QtWidgets.QWidget import width
from pyqtgraph.opengl import *
from pyqtgraph import Transform3D
from PyQt6.QtWidgets import *
#from PyQt5.QtGui import QMatrix4x4
import numpy as np
import typing
from backend.pose import Pose


class Frame:
    """ Simple coordinate frame class. """
    def __init__(self, size=0.050):
        self.x = GLLinePlotItem(pos=np.array([[0,0,0],[size,0,0]]), color=(1,0,0,1), width=2)
        self.y = GLLinePlotItem(pos=np.array([[0,0,0],[0,size,0]]), color=(0,1,0,1), width=2)
        self.z = GLLinePlotItem(pos=np.array([[0,0,0],[0,0,size]]), color=(0,0,1,1), width=2)

    def add_to_view(self, view):
        view.addItem(self.x)
        view.addItem(self.y)
        view.addItem(self.z)

    def translate(self, x, y, z):
        self.x.translate(x,y,z)
        self.y.translate(x,y,z)
        self.z.translate(x,y,z)

    def set_pose(self, pose: Pose):
        # x, y, z = pose.position
        # w, qx, qy, qz = pose.quaternion
        #
        # # Normalize quaternion
        # norm = np.sqrt(w * w + qx * qx + qy * qy + qz * qz)
        #
        # if norm < 1e-10:
        #     return
        #
        # w /= norm
        # qx /= norm
        # qy /= norm
        # qz /= norm
        #
        # rot_mat = [[1 - 2*(qy*qy + qz*qz), 2*(qx*qy - w*qz), 2*(qx*qz + w*qy)],
        #            [2*(qx*qy + w*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - w*qx)],
        #            [2*(qx*qz - w*qy), 2*(qy*qz + w*qx), 1 - 2*(qx*qx + qy*qy)]]
        #
        # transform = np.eye(4)
        # transform[:3, :3] = rot_mat
        # transform[:3, 3] = np.array([x, y, z])

        transform = pose.SE3
        #tr = Transform3D(T)
        self.x.setTransform(transform)
        self.y.setTransform(transform)
        self.z.setTransform(transform)

    def set_major_axis(self, pos: np.ndarray, vals: np.ndarray, vecs: np.ndarray) -> None:
        transform = np.eye(4)
        transform[:3, 3] = pos
        transform[:3, :3] = vecs*vals

        self.x.setTransform(transform)
        self.y.setTransform(transform)
        self.z.setTransform(transform)


class Ellipsoid(GLMeshItem):
    """ Class for drawing manipulability ellipsoid using singular vectors and values """

    def __init__(self, color: tuple, scale: float):

        mesh = MeshData.sphere(rows=20, cols=20)

        super().__init__(meshdata=mesh,
                       smooth=True,
                       color=color,
                       shaders='shaded',
                       glOptions='translucent')

        self._scale = scale
        self._pos = np.zeros(3)
        self._vals = np.zeros(3)
        self._vecs = np.eye(3)

        self._update_transform()

    def set_pose(self, pose: Pose, vals: np.ndarray, vecs: np.ndarray) -> None:
        """ Set posture and scale of ellipsoid
        :param pose:
        :param vals: np.array, (3,) scales the lengths of the semi-principal axes.
        :param vecs: np.array, (3, 3), columns are the normalized singular vectors
        """
        self._pos = np.asarray(pose.position, dtype=float)
        self._vals = np.asarray(vals, dtype=float)
        self._vecs = np.asarray(vecs, dtype=float)
        self._update_transform()

    def set_position(self, pos: np.ndarray) -> None:
        """ Update only the center position """
        self._pos = np.asarray(pos, dtype=float)
        self._update_transform()

    def set_principal_axes(self, vals: np.ndarray, vecs: np.ndarray) -> None:
        """ Update the shape dimensions and orientation simultaneously """
        self._vals = np.asarray(vals, dtype=float)
        self._vecs = np.asarray(vecs, dtype=float)
        self._update_transform()

    def _update_transform(self) -> None:
        """ Internal worker method to calculate and apply the 4x4 affine matrix """
        # SE3 transformation
        transform = np.eye(4)
        transform[:3, :3] = self._vecs * self._scale * self._vals
        transform[:3, 3] = self._pos

        # Apply transform
        self.setTransform(transform)


class View3D(QWidget):

    def __init__(self):

        super().__init__()

        self.view = GLViewWidget()
        self.base_frame = Frame()
        self.tool_frame = Frame()
        self.ellipsoid_trans = Ellipsoid(color=(0.2, 0.6, 1.0, 0.25), scale=0.1)
        self.ellipsoid_rot = Ellipsoid(color=(1.0, 0.6, 0.2, 0.25), scale=0.025)

        self.init_ui()


    def init_ui(self):

        layout = QVBoxLayout()

        layout.addWidget(self.view)

        self.setLayout(layout)

        self.setup_scene()


    def setup_scene(self):

        grid = GLGridItem()
        grid.scale(0.050, 0.050, 0.050)

        self.view.addItem(grid)

        self.base_frame.add_to_view(self.view)
        self.tool_frame.add_to_view(self.view)
        self.view.addItem(self.ellipsoid_trans)
        self.view.addItem(self.ellipsoid_rot)

        self.view.setCameraPosition(distance=1, elevation=30, azimuth=-45)

    def update_status(self, status: dict):
        self.tool_frame.set_pose(status['ops_coords_base'])
        self.ellipsoid_trans.set_pose(status['ops_coords_base'], status['sing_vals_trans'], status['sing_vecs_trans'])
        self.ellipsoid_rot.set_pose(status['ops_coords_base'], status['sing_vals_rot'], status['sing_vecs_rot'])