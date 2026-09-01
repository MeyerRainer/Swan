"""
Class for control panel widget.

Author: Rainer Meyer, r.meyer494@gmail.com
"""
from PyQt6.QtCore import pyqtSignal

import config
from qt_gui.docks.panels.custom_slider import CustomSlider
from config import *

from PyQt6.QtWidgets import *

slider_style = """
    /* The horizontal track background */
    QSlider::groove:horizontal {
        border: none;
        background: #E0E0E0;    /* Light grey track */
        height: 4px;            /* Sleek, thin track line */
        border-radius: 2px;     /* Rounded track edges */
    }

    /* The elegant round handle */
    QSlider::handle:horizontal {
        background: #0078D4;    /* Modern accent blue */
        border: none;
        width: 14px;            /* Width of the circle */
        height: 14px;           /* Height must match width for a perfect circle */
        border-radius: 7px;     /* Radius must be exactly half of width/height */
        margin: -5px 0px;       /* Centers the 14px ball over the 4px track line */
    }

    /* Optional: Change the ball color when hovering for a premium feel */
    QSlider::handle:horizontal:hover {
        background: #005A9E;    /* Darker blue on hover */
    }
"""

class ControlWidget(QWidget):

    def __init__(self):

        super().__init__()

        self.setMinimumWidth(320)

        self.home_button = QPushButton("Home")
        self.enable_button = QPushButton("Enable")
        self.execute_button = QPushButton("Execute Planned")
        self.free_button = QPushButton("")

        self.radio_button_jnt = QRadioButton("Joint")
        self.radio_button_ops = QRadioButton("Cartesian")
        self.radio_button_giz = QRadioButton("Gizmo")
        self.radio_button_prog = QRadioButton("Program")
        self.radio_button_ops.setChecked(True)  # OPS-control by default

        self.frame_select_position = QComboBox()
        self.frame_select_orientation = QComboBox()
        self.frame_select_position.addItems(config.FRAMES)
        self.frame_select_orientation.addItems(config.FRAMES)

        self.increment_linear = QSpinBox()
        self.increment_angular = QSpinBox()
        self.increment_joint_scroll = QSpinBox()
        self.increment_joint_arrow_key = QSpinBox()

        self.group_position = QGroupBox("Position")
        self.x_plus = QPushButton("X+🡆")
        self.x_minus = QPushButton("🡄X-")
        self.y_plus = QPushButton("🡅\nY+")
        self.y_minus = QPushButton("Y-\n🡇")
        self.z_plus = QPushButton("Z+🡅")
        self.z_minus = QPushButton("🡇Z-")
        self.x_plus_y_plus = QPushButton("🢅")
        self.x_plus_y_minus = QPushButton("🢆")
        self.x_minus_y_minus = QPushButton("🢇")
        self.x_minus_y_plus = QPushButton("🢄")
        self.pos_mid = QPushButton("Mid")

        self.group_orientation = QGroupBox("Orientation")
        self.rx_plus = QPushButton("RX+")
        self.rx_minus = QPushButton("RX-")
        self.ry_plus = QPushButton("RY+")
        self.ry_minus = QPushButton("RY-")
        self.rz_plus = QPushButton("RZ+")
        self.rz_minus = QPushButton("RZ-")

        self._jnt_slider_feedback = True

        # Revolute joints
        self.group_joint = QGroupBox("Joints revolute")
        self.j1_slider = CustomSlider("J1", JOINT_LIMITS['J1_MIN'], JOINT_LIMITS['J1_MAX'], 0)
        self.j2_slider = CustomSlider("J2", JOINT_LIMITS['J2_MIN'], JOINT_LIMITS['J2_MAX'], 0)
        self.j3_slider = CustomSlider("J3", JOINT_LIMITS['J3_MIN'], JOINT_LIMITS['J3_MAX'], 0)
        self.j4_slider = CustomSlider("J4", JOINT_LIMITS['J4_MIN'], JOINT_LIMITS['J4_MAX'], 0)
        self.j5_slider = CustomSlider("J5", JOINT_LIMITS['J5_MIN'], JOINT_LIMITS['J5_MAX'], 0)
        self.j6_slider = CustomSlider("J6", JOINT_LIMITS['J6_MIN'], JOINT_LIMITS['J6_MAX'], 0)
        self.j7_slider = CustomSlider("J7", JOINT_LIMITS['J7_MIN'], JOINT_LIMITS['J7_MAX'], 0)
        # Linear joints
        self.group_joint_lin = QGroupBox("Joints Linear")
        self.jl1_slider = CustomSlider("L1", JOINT_LINEAR_LIMITS['JL1_MIN'], JOINT_LINEAR_LIMITS['JL1_MAX'], 0)
        self.jl2_slider = CustomSlider("L2", JOINT_LINEAR_LIMITS['JL2_MIN'], JOINT_LINEAR_LIMITS['JL2_MAX'], 0)

        # Nullspace
        self.group_nullspace = QGroupBox("Nullspace")
        self.null1_slider = CustomSlider("N1", JOINT_LINEAR_LIMITS['JL1_MIN'], JOINT_LINEAR_LIMITS['JL1_MAX'], 0)
        self.null2_slider = CustomSlider("N2", JOINT_LINEAR_LIMITS['JL2_MIN'], JOINT_LINEAR_LIMITS['JL2_MAX'], 0)

        # Speeds
        self.speed_linear = CustomSlider("Linear (mm/min)  ", LINEAR_SPEED_MIN, LINEAR_SPEED_MAX, LINEAR_SPEED_DEFAULT)
        self.speed_angular = CustomSlider("Angular (deg/min)", ANGULAR_SPEED_MIN, ANGULAR_SPEED_MAX, ANGULAR_SPEED_DEFAULT)
        self.speed_joint = CustomSlider("Joint (deg/min) ", JOINT_SPEED_MIN, JOINT_SPEED_MAX, JOINT_SPEED_DEFAULT)

        self.switch_ops_mode(True)
        self.init_ui()

    @property
    def joint_slider_feedback(self):
        return self._jnt_slider_feedback

    def init_ui(self):

        layout = QVBoxLayout()

        # ======================================= Control Mode =======================================
        motion_group = QGroupBox("Motion")
        v_motion_layout = QVBoxLayout()

        g_option_layout = QGridLayout()
        g_option_layout.addWidget(self.home_button, 0, 0)
        g_option_layout.addWidget(self.enable_button, 0, 1)
        g_option_layout.addWidget(self.execute_button, 1, 0)
        g_option_layout.addWidget(self.free_button, 1, 1)

        v_motion_layout.addLayout(g_option_layout)

        self.radio_button_jnt.toggled.connect(self.switch_joint_mode)
        self.radio_button_ops.toggled.connect(self.switch_ops_mode)
        self.radio_button_giz.toggled.connect(self.switch_gizmo_mode)
        self.radio_button_prog.toggled.connect(self.switch_program_mode)
        h_layout_control_mode = QHBoxLayout()
        h_layout_control_mode.addWidget(self.radio_button_jnt)
        h_layout_control_mode.addWidget(self.radio_button_ops)
        h_layout_control_mode.addWidget(self.radio_button_giz)
        h_layout_control_mode.addWidget(self.radio_button_prog)

        v_motion_layout.addLayout(h_layout_control_mode)

        motion_group.setLayout(v_motion_layout)

        # ======================================= OPS Control ========================================
        h_layout_ops = QHBoxLayout()

        # Position
        v_layout_position = QVBoxLayout()
        g_layout_position = QGridLayout()

        g_layout_position.addWidget(self.x_plus, 1, 2)
        g_layout_position.addWidget(self.x_minus, 1, 0)
        g_layout_position.addWidget(self.y_plus, 0 , 1)
        g_layout_position.addWidget(self.y_minus, 2, 1)
        g_layout_position.addWidget(self.z_plus, 3, 2)
        g_layout_position.addWidget(self.z_minus, 3, 0)
        g_layout_position.addWidget(self.x_plus_y_plus, 0, 2)
        g_layout_position.addWidget(self.x_plus_y_minus, 2, 2)
        g_layout_position.addWidget(self.x_minus_y_minus, 2, 0)
        g_layout_position.addWidget(self.x_minus_y_plus, 0, 0)
        g_layout_position.addWidget(self.pos_mid, 1, 1)

        v_layout_position.addLayout(g_layout_position)

        self.increment_linear.setValue(LINEAR_INCREMENT)
        g_layout_increment = QGridLayout()
        g_layout_increment.addWidget(QLabel("Increment (mm)"), 0, 0)
        g_layout_increment.addWidget(self.increment_linear, 0, 1)
        g_layout_increment.addWidget(QLabel("Frame"), 1, 0)
        g_layout_increment.addWidget(self.frame_select_position, 1, 1)
        v_layout_position.addLayout(g_layout_increment)

        self.group_position.setLayout(v_layout_position)

        # Orientation
        g_layout_orientation = QGridLayout()

        g_layout_orientation.addWidget(self.rx_minus, 0, 0)
        g_layout_orientation.addWidget(self.rx_plus, 0, 1)
        g_layout_orientation.addWidget(self.ry_minus, 1, 0)
        g_layout_orientation.addWidget(self.ry_plus, 1 , 1)
        g_layout_orientation.addWidget(self.rz_minus, 2, 0)
        g_layout_orientation.addWidget(self.rz_plus, 2, 1)

        # Increments
        self.increment_angular.setValue(ANGULAR_INCREMENT)
        g_layout_orientation.addWidget(QLabel("Increment"), 3, 0)
        g_layout_orientation.addWidget(self.increment_angular, 3, 1)
        g_layout_orientation.addWidget(QLabel("Frame"), 4, 0)
        g_layout_orientation.addWidget(self.frame_select_orientation, 4, 1)
        self.group_orientation.setLayout(g_layout_orientation)

        h_layout_ops.addWidget(self.group_orientation)
        h_layout_ops.addWidget(self.group_position)

        # ======================================= Joint control =======================================
        v_layout_joint = QVBoxLayout()

        v_layout_joint.addWidget(self.j1_slider)
        self.j1_slider.set_steps(JOINT_INCREMENT_ARROW_KEY, JOINT_INCREMENT_SCROLL)

        v_layout_joint.addWidget(self.j2_slider)
        self.j2_slider.set_steps(JOINT_INCREMENT_ARROW_KEY, JOINT_INCREMENT_SCROLL)

        v_layout_joint.addWidget(self.j3_slider)
        self.j3_slider.set_steps(JOINT_INCREMENT_ARROW_KEY, JOINT_INCREMENT_SCROLL)

        v_layout_joint.addWidget(self.j4_slider)
        self.j4_slider.set_steps(JOINT_INCREMENT_ARROW_KEY, JOINT_INCREMENT_SCROLL)

        v_layout_joint.addWidget(self.j5_slider)
        self.j5_slider.set_steps(JOINT_INCREMENT_ARROW_KEY, JOINT_INCREMENT_SCROLL)

        v_layout_joint.addWidget(self.j6_slider)
        self.j6_slider.set_steps(JOINT_INCREMENT_ARROW_KEY, JOINT_INCREMENT_SCROLL)

        v_layout_joint.addWidget(self.j7_slider)
        self.j7_slider.set_steps(JOINT_INCREMENT_ARROW_KEY, JOINT_INCREMENT_SCROLL)

        self.group_joint.setLayout(v_layout_joint)
        v_layout_joint_lin = QVBoxLayout()

        # Linear joints
        self.jl1_slider.set_steps(JOINT_INCREMENT_ARROW_KEY, JOINT_INCREMENT_SCROLL)
        v_layout_joint_lin.addWidget(self.jl1_slider)

        self.jl2_slider.set_steps(JOINT_INCREMENT_ARROW_KEY, JOINT_INCREMENT_SCROLL)
        v_layout_joint_lin.addWidget(self.jl2_slider)

        self.group_joint_lin.setLayout(v_layout_joint_lin)


        # ======================================= Nullspace =======================================
        v_layout_nullspace = QVBoxLayout()

        v_layout_nullspace.addWidget(self.null1_slider)
        self.null1_slider.set_steps(5, 1)

        v_layout_nullspace.addWidget(self.null2_slider)
        self.null2_slider.set_steps(5, 1)

        self.group_nullspace.setLayout(v_layout_nullspace)

        # ========================================= Increment =========================================
        group_increment = QGroupBox("Joint Increment")
        g_layout_increment = QHBoxLayout()
        # Increment Scroll
        self.increment_joint_scroll.setValue(JOINT_INCREMENT_SCROLL)
        g_layout_increment.addWidget(QLabel("Scroll"))
        g_layout_increment.addWidget(self.increment_joint_scroll)
        self.increment_joint_scroll.valueChanged.connect(self.update_slider_increments)
        # Increment Arrow key
        self.increment_joint_arrow_key.setValue(JOINT_INCREMENT_ARROW_KEY)
        g_layout_increment.addWidget(QLabel("Arrow key"))
        g_layout_increment.addWidget(self.increment_joint_arrow_key)
        group_increment.setLayout(g_layout_increment)
        self.increment_joint_arrow_key.valueChanged.connect(self.update_slider_increments)

        # ======================================== Speeds =========================================
        group_speed = QGroupBox("Speed")
        v_layout_speed = QVBoxLayout()

        v_layout_speed.addWidget(self.speed_linear)
        self.speed_linear.set_steps(100, 20)

        v_layout_speed.addWidget(self.speed_angular)
        self.speed_angular.set_steps(100, 20)

        v_layout_speed.addWidget(self.speed_joint)
        self.speed_joint.set_steps(100, 20)

        group_speed.setLayout(v_layout_speed)

        # ================================ Assemble control panel =================================
        layout.addWidget(motion_group)
        layout.addLayout(h_layout_ops)
        layout.addWidget(self.group_joint)
        layout.addWidget(self.group_joint_lin)
        layout.addWidget(self.group_nullspace)
        layout.addWidget(group_increment)
        layout.addWidget(group_speed)
        layout.addStretch()

        self.setLayout(layout)

    def update_status(self, status_dict: dict):
        joint_values_deg = status_dict['jnt_coords_deg']
        self.update_joint_sliders(joint_values_deg)

    def update_joint_sliders(self, jnt_list: np.ndarray):
        """ Update joint sliders if robot has moved
        @param jnt_list: List of joint values
        """
        if self._jnt_slider_feedback:
            self.j1_slider.set_value(jnt_list[0])
            self.j2_slider.set_value(jnt_list[1])
            self.j3_slider.set_value(jnt_list[2])
            self.j4_slider.set_value(jnt_list[3])
            self.j5_slider.set_value(jnt_list[4])
            self.j6_slider.set_value(jnt_list[5])

            self.jl1_slider.set_value(jnt_list[6])


    def cut_joint_slider_feedback(self):
        self._jnt_slider_feedback = False

    def regain_joint_slider_feedback(self):
        self._jnt_slider_feedback = True

    def update_slider_increments(self):
        """ Update scrolling and arrow key increments on the sliders """
        self.j1_slider.set_steps(int(self.increment_joint_arrow_key.text()), int(self.increment_joint_scroll.text()))
        self.j2_slider.set_steps(int(self.increment_joint_arrow_key.text()), int(self.increment_joint_scroll.text()))
        self.j3_slider.set_steps(int(self.increment_joint_arrow_key.text()), int(self.increment_joint_scroll.text()))
        self.j4_slider.set_steps(int(self.increment_joint_arrow_key.text()), int(self.increment_joint_scroll.text()))
        self.j5_slider.set_steps(int(self.increment_joint_arrow_key.text()), int(self.increment_joint_scroll.text()))
        self.j6_slider.set_steps(int(self.increment_joint_arrow_key.text()), int(self.increment_joint_scroll.text()))
        self.j7_slider.set_steps(int(self.increment_joint_arrow_key.text()), int(self.increment_joint_scroll.text()))

        self.jl1_slider.set_steps(int(self.increment_joint_arrow_key.text()), int(self.increment_joint_scroll.text()))
        self.jl2_slider.set_steps(int(self.increment_joint_arrow_key.text()), int(self.increment_joint_scroll.text()))


    def joint_panel_active(self, on: bool) -> None:
        if on:
            self.cut_joint_slider_feedback()
            self.group_joint.setEnabled(True)
            self.group_joint_lin.setEnabled(True)
            self.group_nullspace.setEnabled(True)
        else:
            self.regain_joint_slider_feedback()
            self.group_joint.setEnabled(False)
            self.group_joint_lin.setEnabled(False)
            self.group_nullspace.setEnabled(False)

    def ops_panel_active(self, on: bool) -> None:
        if on:
            self.group_position.setEnabled(True)
            self.group_orientation.setEnabled(True)
        else:
            self.group_position.setEnabled(False)
            self.group_orientation.setEnabled(False)

    def switch_joint_mode(self, checked):
        if checked:
            self.joint_panel_active(True)
            self.ops_panel_active(False)

    def switch_ops_mode(self, checked: bool):
        if checked:
            self.joint_panel_active(False)
            self.ops_panel_active(True)

    def switch_gizmo_mode(self, checked: bool) -> None:
        if checked:
            self.joint_panel_active(False)
            self.ops_panel_active(False)

    def switch_program_mode(self, checked: bool) -> None:
        if checked:
            self.joint_panel_active(False)
            self.ops_panel_active(False)
