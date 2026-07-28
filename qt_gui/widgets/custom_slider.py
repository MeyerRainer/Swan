from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QSlider
from PyQt6.QtCore import Qt

STYLE = """
    /* ================= ACTIVE STATE ================= */

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
        width: 12px;            /* Width of the circle */
        height: 12px;           /* Height must match width for a perfect circle */
        border-radius: 6px;     /* Radius must be exactly half of width/height */
        margin: -4px 0px;       /* Centers the 12px ball over the 4px track line */
    }

    /* Change the ball color when hovering */
    QSlider::handle:horizontal:hover {
        background: #005A9E;    /* Darker blue on hover */
    }

    /* ================= DISABLED STATE ================= */

    /* Subdued track when disabled */
    QSlider::groove:horizontal:disabled {
        background: #F0F0F0;    /* Very light, washed-out grey */
    }

    /* Muted handle when disabled */
    QSlider::handle:horizontal:disabled {
        background: #CCCCCC;    /* Muted, neutral grey handle */
    }
"""

class CustomSlider(QWidget):
    """
    A reusable widget containing:
    [Left Label] --- [Slider] --- [Right Value Label]
    """

    def __init__(self, name, min_val: int = 0, max_val: int = 100, default_val: int = 10, parent=None):

        super().__init__(parent)

        # Internal state to track dragging
        self._is_dragging = False
        self.callback = None

        # Setup Layout
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)  # Tight alignment

        # 1. Left Name Label
        self.name_label = QLabel(name, self)
        layout.addWidget(self.name_label)

        # 2. The Slider
        self.slider = QSlider(Qt.Orientation.Horizontal, self)
        self.slider.setMaximumHeight(14)
        self.slider.setStyleSheet(STYLE)
        self.slider.setMinimum(min_val)
        self.slider.setMaximum(max_val)
        self.slider.setValue(default_val)
        layout.addWidget(self.slider)

        # 3. Right Value Label (Updates continuously)
        self.value_label = QLabel(str(self.slider.value()), self)
        self.value_label.setFixedWidth(40)  # Prevent UI shifting as numbers change
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.value_label)

        self.setLayout(layout)

        # Connect internal signals for live label updates and drag tracking
        self.slider.sliderPressed.connect(self._on_pressed)
        self.slider.sliderReleased.connect(self._on_released)
        self.slider.valueChanged.connect(self._on_value_changed)

    def value(self):
        return self.slider.value()

    def set_steps(self, single_step, page_step):
        """Easily configure step sizes from external widgets or logic."""
        self.slider.setSingleStep(single_step)
        self.slider.setPageStep(page_step)

    def connect_target(self, callback_func):
        """Registers the external function you want to execute."""
        self.callback = callback_func

    def set_value(self, value: float):
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(value)))
        self.value_label.setText("{:.2f}".format(value))
        self.slider.blockSignals(False)

    def set_enabled(self, enabled: bool):
        self.slider.setEnabled(enabled)

    def _on_pressed(self):
        self._is_dragging = True

    def _on_released(self):
        self._is_dragging = False
        # 3. Dragging release: trigger heavy function once
        if self.callback:
            self.callback(self.slider.value())

    def _on_value_changed(self, value):
        # Always update the right side text immediately
        self.value_label.setText(str(value))

        # 1 & 2: Arrow keys/Scrolling trigger function immediately (if not dragging)
        if not self._is_dragging and self.callback:
            self.callback(value)