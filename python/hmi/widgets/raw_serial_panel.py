from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
)


class RawSerialPanel(QGroupBox):
    """Panel for sending one-line JSON commands directly over serial."""

    command_requested = Signal(str)

    def __init__(self) -> None:
        super().__init__("Raw Serial Command")

        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText(
            '{"cmd":"MOVE_ACT","z_dir":0,"z_time_s":0,"s2_deg":0,"s3_deg":180}'
        )

        self.preset_combo = QComboBox()
        self.preset_combo.setMinimumContentsLength(12)
        self.preset_combo.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.preset_combo.addItem("Custom", "")
        self.preset_combo.addItem("PING", '{"cmd":"PING"}')
        self.preset_combo.addItem("HOME_Z", '{"cmd":"HOME_Z"}')
        self.preset_combo.addItem("Route HOME", '{"cmd":"HOME"}')
        self.preset_combo.addItem(
            "MOVE_ACT home",
            '{"cmd":"MOVE_ACT","z_dir":0,"z_time_s":0,"s2_deg":0,"s3_deg":180}',
        )

        self.send_button = QPushButton("Send")
        self.clear_button = QPushButton("Clear")

        self._build_ui()
        self._connect_signals()

    def _build_ui(self) -> None:
        layout = QVBoxLayout()
        input_layout = QHBoxLayout()
        button_layout = QHBoxLayout()

        safety_label = QLabel("Sends one JSON object per line. Use only while the robot is in a safe state.")
        safety_label.setWordWrap(True)

        input_layout.addWidget(QLabel("JSON:"))
        input_layout.addWidget(self.command_input, stretch=1)

        button_layout.addWidget(QLabel("Preset:"))
        button_layout.addWidget(self.preset_combo, stretch=1)
        button_layout.addWidget(self.send_button)
        button_layout.addWidget(self.clear_button)

        layout.addLayout(input_layout)
        layout.addLayout(button_layout)
        layout.addWidget(safety_label)

        self.setLayout(layout)

    def _connect_signals(self) -> None:
        self.send_button.clicked.connect(self._emit_command)
        self.clear_button.clicked.connect(self.command_input.clear)
        self.command_input.returnPressed.connect(self._emit_command)
        self.preset_combo.currentIndexChanged.connect(self._on_preset_changed)

    def _on_preset_changed(self) -> None:
        command = self.preset_combo.currentData()
        if command:
            self.command_input.setText(command)

    def _emit_command(self) -> None:
        command = self.command_input.text().strip()
        if command:
            self.command_requested.emit(command)

    def set_enabled(self, enabled: bool) -> None:
        self.command_input.setEnabled(enabled)
        self.preset_combo.setEnabled(enabled)
        self.send_button.setEnabled(enabled)
        self.clear_button.setEnabled(enabled)
