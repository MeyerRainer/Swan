"""
Class for digital readout widget
Author: Rainer Meyer, rot.meyer494@gmail.com
"""
import numpy as np
from PyQt6.QtWidgets import QWidget, QGridLayout, QLabel, QGroupBox, QVBoxLayout, QHBoxLayout, QComboBox
import config


class DROWidget(QWidget):

    def __init__(self):

        super().__init__()

        self.frame_select_position = QComboBox()
        self.frame_select_position.addItems(config.FRAMES)
        self.frame_select_orientation = QComboBox()
        self.frame_select_orientation.addItems(config.FRAMES)
        self.frame_select_condition = QComboBox()
        self.frame_select_condition.addItems(config.FRAMES)
        self.frame_select_position.setCurrentText(config.FRAMES[1])
        self.frame_select_orientation.setCurrentText(config.FRAMES[1])
        self.frame_select_condition.setCurrentText(config.FRAMES[1])

        # Position
        self.position_x = QLabel("0.000")
        self.position_y = QLabel("0.000")
        self.position_z = QLabel("0.000")

        # ZYZ-Euler angles
        self.zyz_euler_1 = QLabel("0.000")
        self.zyz_euler_2 = QLabel("0.000")
        self.zyz_euler_3 = QLabel("0.000")

        # Quaternion
        self.quaternion_w = QLabel("0.000")
        self.quaternion_i = QLabel("0.000")
        self.quaternion_j = QLabel("0.000")
        self.quaternion_k = QLabel("0.000")

        self.condition_x = QLabel("0.000")
        self.condition_y = QLabel("0.000")
        self.condition_z = QLabel("0.000")
        self.condition_rx = QLabel("0.000")
        self.condition_ry = QLabel("0.000")
        self.condition_rz = QLabel("0.000")

        self.jnt1 = QLabel("0.000")
        self.jnt2 = QLabel("0.000")
        self.jnt3 = QLabel("0.000")
        self.jnt4 = QLabel("0.000")
        self.jnt5 = QLabel("0.000")
        self.jnt6 = QLabel("0.000")
        self.jnt7 = QLabel("0.000")
        self.jnt8 = QLabel("0.000")

        self.mot1 = QLabel("0.000")
        self.mot2 = QLabel("0.000")
        self.mot3 = QLabel("0.000")
        self.mot4 = QLabel("0.000")
        self.mot5 = QLabel("0.000")
        self.mot6 = QLabel("0.000")
        self.mot7 = QLabel("0.000")
        self.mot8 = QLabel("0.000")

        self.set_fonts()
        self.init_ui()


    def init_ui(self):

        layout = QVBoxLayout()
        lower = QHBoxLayout()
        self.setMinimumWidth(360)

        # ==================================== Position =======================================
        group_position = QGroupBox("Position")
        v_layout_position = QVBoxLayout()
        frame_select_position = QHBoxLayout()
        frame_select_position.addWidget(QLabel("Frame"))
        frame_select_position.addWidget(self.frame_select_position)
        v_layout_position.addLayout(frame_select_position)

        display_position = QGridLayout()
        display_position.addWidget(QLabel("X"), 0, 0)
        display_position.addWidget(self.position_x, 1, 0)
        display_position.addWidget(QLabel("Y"), 0, 1)
        display_position.addWidget(self.position_y, 1, 1)
        display_position.addWidget(QLabel("Z"), 0, 2)
        display_position.addWidget(self.position_z, 1, 2)
        v_layout_position.addLayout(display_position)

        group_position.setLayout(v_layout_position)

        # ==================================== Orientation =======================================
        group_orientation = QGroupBox("Orientation")

        v_layout_orientation = QVBoxLayout()

        h_layout_orientation_frames = QHBoxLayout()
        g_layout_orientation = QGridLayout()

        # Frame select
        h_layout_orientation_frames.addWidget(QLabel("Frame"))
        h_layout_orientation_frames.addWidget(self.frame_select_orientation)
        v_layout_orientation.addLayout(h_layout_orientation_frames)

        # Quaternion
        g_layout_orientation.addWidget(QLabel("Quaternion\nW I J K"), 1, 0)
        display_quaternion = QHBoxLayout()
        display_quaternion.addWidget(self.quaternion_w)
        display_quaternion.addWidget(self.quaternion_i)
        display_quaternion.addWidget(self.quaternion_j)
        display_quaternion.addWidget(self.quaternion_k)
        g_layout_orientation.addLayout(display_quaternion, 1, 1)

        # --- ZYZ-Euler ---
        g_layout_orientation.addWidget(QLabel("Euler\nZ Y Z"), 0, 0)
        display_zyz = QHBoxLayout()
        display_zyz.addWidget(self.zyz_euler_1)
        display_zyz.addWidget(self.zyz_euler_2)
        display_zyz.addWidget(self.zyz_euler_3)
        g_layout_orientation.addLayout(display_zyz, 0, 1)

        v_layout_orientation.addLayout(g_layout_orientation)
        group_orientation.setLayout(v_layout_orientation)

        # ==================================== Joint & Motor =======================================
        group_jnt_mot = QGroupBox("Joint Motor")
        g_layout_jnt_mot = QGridLayout()

        g_layout_jnt_mot.addWidget(QLabel("1"), 0, 0)
        g_layout_jnt_mot.addWidget(self.jnt1, 0, 1)
        g_layout_jnt_mot.addWidget(self.mot1, 0, 2)
        g_layout_jnt_mot.addWidget(QLabel("2"), 1, 0)
        g_layout_jnt_mot.addWidget(self.jnt2, 1, 1)
        g_layout_jnt_mot.addWidget(self.mot2, 1, 2)
        g_layout_jnt_mot.addWidget(QLabel("3"), 2, 0)
        g_layout_jnt_mot.addWidget(self.jnt3, 2, 1)
        g_layout_jnt_mot.addWidget(self.mot3, 2, 2)
        g_layout_jnt_mot.addWidget(QLabel("4"), 3, 0)
        g_layout_jnt_mot.addWidget(self.jnt4, 3, 1)
        g_layout_jnt_mot.addWidget(self.mot4, 3, 2)
        g_layout_jnt_mot.addWidget(QLabel("5"), 4, 0)
        g_layout_jnt_mot.addWidget(self.jnt5, 4, 1)
        g_layout_jnt_mot.addWidget(self.mot5, 4, 2)
        g_layout_jnt_mot.addWidget(QLabel("6"), 5, 0)
        g_layout_jnt_mot.addWidget(self.jnt6, 5, 1)
        g_layout_jnt_mot.addWidget(self.mot6, 5, 2)
        g_layout_jnt_mot.addWidget(QLabel("7"), 6, 0)
        g_layout_jnt_mot.addWidget(self.jnt7, 6, 1)
        g_layout_jnt_mot.addWidget(self.mot7, 6, 2)
        g_layout_jnt_mot.addWidget(QLabel("8"), 7, 0)
        g_layout_jnt_mot.addWidget(self.jnt8, 7, 1)
        g_layout_jnt_mot.addWidget(self.mot8, 7, 2)
        group_jnt_mot.setLayout(g_layout_jnt_mot)

        # ==================================== Condition =======================================
        group_condition = QGroupBox("Condition")
        v_layout_condition = QVBoxLayout()
        v_layout_condition.addWidget(QLabel("Frame"))
        v_layout_condition.addWidget(self.frame_select_condition)


        g_display_condition = QGridLayout()
        g_display_condition.addWidget(QLabel("X"), 0, 0)
        g_display_condition.addWidget(self.condition_x, 0, 1)
        g_display_condition.addWidget(QLabel("Y"), 1, 0)
        g_display_condition.addWidget(self.condition_y, 1, 1)
        g_display_condition.addWidget(QLabel("Z"), 2, 0)
        g_display_condition.addWidget(self.condition_z, 2, 1)
        g_display_condition.addWidget(QLabel("RX"), 3, 0)
        g_display_condition.addWidget(self.condition_rx, 3, 1)
        g_display_condition.addWidget(QLabel("RY"), 4, 0)
        g_display_condition.addWidget(self.condition_ry, 4, 1)
        g_display_condition.addWidget(QLabel("RZ"), 5, 0)
        g_display_condition.addWidget(self.condition_rz, 5, 1)
        v_layout_condition.addLayout(g_display_condition)

        group_condition.setLayout(v_layout_condition)


        # Upper
        layout.addWidget(group_position)
        layout.addWidget(group_orientation)

        # Lower
        lower.addWidget(group_condition)
        lower.addWidget(group_jnt_mot)

        layout.addLayout(lower)

        layout.addStretch()

        self.setLayout(layout)


    def update_status(self, status: dict):
        # Position
        frame_position = self.frame_select_position.currentText()
        if frame_position == 'World':
            pos_vec: np.ndarray = status['ops_coords_world'][:3]
        elif frame_position == "Base":
            pos_vec: np.ndarray = status['ops_coords_base'][:3]
        elif frame_position == "Tool":
            pos_vec = np.zeros(3)
        else:
            raise ValueError("Invalid frame")

        self.position_x.setText(f"{pos_vec[0]:.3f}")
        self.position_y.setText(f"{pos_vec[1]:.3f}")
        self.position_z.setText(f"{pos_vec[2]:.3f}")

        # Orientation
        frame_orientation = self.frame_select_orientation.currentText()
        if frame_orientation == 'World':
            quaternion: np.ndarray = status['ops_coords_world'][3:]
            zyz: np.ndarray = status['zyz_euler_world']
        elif frame_orientation == "Base":
            quaternion: np.ndarray = status['ops_coords_base'][3:]
            zyz: np.ndarray = status['zyz_euler_base']
        elif frame_orientation == "Tool":
            quaternion = np.array((1., 0., 0., 0.))
            zyz: np.ndarray = np.zeros(3)
        else:
            raise ValueError("Invalid frame")

        self.quaternion_w.setText(f"{quaternion[0]:.3f}")
        self.quaternion_i.setText(f"{quaternion[1]:.3f}")
        self.quaternion_j.setText(f"{quaternion[2]:.3f}")
        self.quaternion_k.setText(f"{quaternion[3]:.3f}")

        self.zyz_euler_1.setText(f"{zyz[0]:.3f}")
        self.zyz_euler_2.setText(f"{zyz[1]:.3f}")
        self.zyz_euler_3.setText(f"{zyz[2]:.3f}")

        # Condition
        frame_condition = self.frame_select_condition.currentText()
        if frame_condition == 'World':
            condition = status['condition_world']
        elif frame_condition == "Base":
            condition = status['condition_base']
        elif frame_condition == "Tool":
            condition = status['condition_tool']
        else:
            raise ValueError("Invalid frame")
        self.condition_x.setText(f"{condition[0]:.2f}")
        self.condition_y.setText(f"{condition[1]:.2f}")
        self.condition_z.setText(f"{condition[2]:.2f}")
        self.condition_rx.setText(f"{condition[3]:.2f}")
        self.condition_ry.setText(f"{condition[4]:.2f}")
        self.condition_rz.setText(f"{condition[5]:.2f}")

        # Joints
        self.jnt1.setText(f"{status['jnt_coords_deg'][0]:.2f}")
        self.jnt2.setText(f"{status['jnt_coords_deg'][1]:.2f}")
        self.jnt3.setText(f"{status['jnt_coords_deg'][2]:.2f}")
        self.jnt4.setText(f"{status['jnt_coords_deg'][3]:.2f}")
        self.jnt5.setText(f"{status['jnt_coords_deg'][4]:.2f}")
        self.jnt6.setText(f"{status['jnt_coords_deg'][5]:.2f}")
        self.jnt7.setText(f"{status['jnt_coords_deg'][6]:.2f}")
        self.jnt8.setText(f"{status['jnt_coords_deg'][7]:.2f}")

        # Motors
        self.mot1.setText(f"{status['mot_coords_deg'][0]:.2f}")
        self.mot2.setText(f"{status['mot_coords_deg'][1]:.2f}")
        self.mot3.setText(f"{status['mot_coords_deg'][2]:.2f}")
        self.mot4.setText(f"{status['mot_coords_deg'][3]:.2f}")
        self.mot5.setText(f"{status['mot_coords_deg'][4]:.2f}")
        self.mot6.setText(f"{status['mot_coords_deg'][5]:.2f}")
        self.mot7.setText(f"{status['mot_coords_deg'][6]:.2f}")
        self.mot8.setText(f"{status['mot_coords_deg'][7]:.2f}")

    def set_fonts(self):
        font_position = self.font()
        font_position.setPointSize(18)

        font_quaternion = self.font()
        font_quaternion.setPointSize(15)

        font_zyz = self.font()
        font_zyz.setPointSize(18)

        font_condition = self.font()
        font_condition.setPointSize(14)

        font_jnt_mot = self.font()
        font_jnt_mot.setPointSize(14)

        self.position_x.setFont(font_position)
        self.position_y.setFont(font_position)
        self.position_z.setFont(font_position)

        self.quaternion_w.setFont(font_quaternion)
        self.quaternion_i.setFont(font_quaternion)
        self.quaternion_j.setFont(font_quaternion)
        self.quaternion_k.setFont(font_quaternion)

        self.zyz_euler_1.setFont(font_zyz)
        self.zyz_euler_2.setFont(font_zyz)
        self.zyz_euler_3.setFont(font_zyz)

        self.condition_x.setFont(font_condition)
        self.condition_y.setFont(font_condition)
        self.condition_z.setFont(font_condition)
        self.condition_rx.setFont(font_condition)
        self.condition_ry.setFont(font_condition)
        self.condition_rz.setFont(font_condition)

        self.jnt1.setFont(font_jnt_mot)
        self.jnt2.setFont(font_jnt_mot)
        self.jnt3.setFont(font_jnt_mot)
        self.jnt4.setFont(font_jnt_mot)
        self.jnt5.setFont(font_jnt_mot)
        self.jnt6.setFont(font_jnt_mot)
        self.jnt7.setFont(font_jnt_mot)
        self.jnt8.setFont(font_jnt_mot)

        self.mot1.setFont(font_jnt_mot)
        self.mot2.setFont(font_jnt_mot)
        self.mot3.setFont(font_jnt_mot)
        self.mot4.setFont(font_jnt_mot)
        self.mot5.setFont(font_jnt_mot)
        self.mot6.setFont(font_jnt_mot)
        self.mot7.setFont(font_jnt_mot)
        self.mot8.setFont(font_jnt_mot)