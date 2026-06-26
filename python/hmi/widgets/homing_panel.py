from PySide6.QtCore import Signal
from PySide6.QtWidgets import QGroupBox, QVBoxLayout, QPushButton, QLabel


class HomingPanel(QGroupBox):
    home_requested = Signal()
    stop_requested = Signal()
    estop_requested = Signal()

    def __init__(self) -> None:
        super().__init__("Homing and Safety")

        self.home_button = QPushButton("Home Robot")
        self.stop_button = QPushButton("Stop")
        self.estop_button = QPushButton("Emergency Stop")

        self.homed_label = QLabel("Homed: No")
        self.state_label = QLabel("State: Idle")

        self._build_ui()
        self._connect_signals()

    def _build_ui(self) -> None:
        layout = QVBoxLayout()

        layout.addWidget(self.home_button)
        layout.addWidget(self.stop_button)
        layout.addWidget(self.estop_button)
        layout.addWidget(self.homed_label)
        layout.addWidget(self.state_label)

        self.setLayout(layout)

    def _connect_signals(self) -> None:
        self.home_button.clicked.connect(self.home_requested.emit)
        self.stop_button.clicked.connect(self.stop_requested.emit)
        self.estop_button.clicked.connect(self.estop_requested.emit)