from PySide6.QtCore import Signal
from PySide6.QtWidgets import QGroupBox, QVBoxLayout, QPushButton, QLabel


class HomingPanel(QGroupBox):
    initial_home_requested = Signal()
    home_requested = Signal()
    stop_requested = Signal()
    estop_requested = Signal()

    def __init__(self) -> None:
        super().__init__("Homing and Safety")

        self.initial_home_button = QPushButton("Initial Z Homing")
        self.home_button = QPushButton("Route Home")
        self.stop_button = QPushButton("Stop")
        self.estop_button = QPushButton("Emergency Stop")

        self.homed_label = QLabel("Z Calibrated: No")
        self.state_label = QLabel("State: Idle")

        self._build_ui()
        self._connect_signals()

    def _build_ui(self) -> None:
        layout = QVBoxLayout()

        layout.addWidget(self.initial_home_button)
        layout.addWidget(self.home_button)
        layout.addWidget(self.stop_button)
        layout.addWidget(self.estop_button)
        layout.addWidget(self.homed_label)
        layout.addWidget(self.state_label)

        self.setLayout(layout)

    def _connect_signals(self) -> None:
        self.initial_home_button.clicked.connect(self.initial_home_requested.emit)
        self.home_button.clicked.connect(self.home_requested.emit)
        self.stop_button.clicked.connect(self.stop_requested.emit)
        self.estop_button.clicked.connect(self.estop_requested.emit)
