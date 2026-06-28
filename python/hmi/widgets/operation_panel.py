from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QGroupBox,
    QVBoxLayout,
    QRadioButton,
    QPushButton,
    QLabel,
)


class OperationPanel(QGroupBox):
    start_requested = Signal()

    def __init__(self) -> None:
        super().__init__("Operation")

        self.all_wells_radio = QRadioButton("Transfer to all wells")
        self.selected_wells_radio = QRadioButton("Transfer to selected wells")
        self.fixed_volume_label = QLabel("Tool volume: fixed 1.0 ml per trip")

        self.all_wells_radio.setChecked(True)

        self.start_button = QPushButton("Start Process")

        self._build_ui()
        self._connect_signals()

    def _build_ui(self) -> None:
        layout = QVBoxLayout()

        layout.addWidget(self.all_wells_radio)
        layout.addWidget(self.selected_wells_radio)
        layout.addWidget(self.fixed_volume_label)
        layout.addWidget(self.start_button)

        self.setLayout(layout)

    def _connect_signals(self) -> None:
        self.start_button.clicked.connect(self.start_requested.emit)

    def get_selected_mode(self) -> str:
        if self.all_wells_radio.isChecked():
            return "all"
        return "selected"

    def get_volume_ml(self) -> float:
        return 1.0
