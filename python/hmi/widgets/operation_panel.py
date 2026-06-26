from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QGroupBox,
    QVBoxLayout,
    QRadioButton,
    QPushButton,
    QDoubleSpinBox,
    QLabel,
)


class OperationPanel(QGroupBox):
    start_requested = Signal()

    def __init__(self) -> None:
        super().__init__("Operation")

        self.all_wells_radio = QRadioButton("Transfer to all wells")
        self.selected_wells_radio = QRadioButton("Transfer to selected wells")
        self.route_radio = QRadioButton("Route")

        self.all_wells_radio.setChecked(True)

        self.volume_spin = QDoubleSpinBox()
        self.volume_spin.setRange(0.1, 3.0)
        self.volume_spin.setSingleStep(0.1)
        self.volume_spin.setValue(1.0)
        self.volume_spin.setSuffix(" ml")

        self.start_button = QPushButton("Start Process")

        self._build_ui()
        self._connect_signals()

    def _build_ui(self) -> None:
        layout = QVBoxLayout()

        layout.addWidget(self.all_wells_radio)
        layout.addWidget(self.selected_wells_radio)
        layout.addWidget(self.route_radio)

        layout.addWidget(QLabel("Volume:"))
        layout.addWidget(self.volume_spin)

        layout.addWidget(self.start_button)

        self.setLayout(layout)

    def _connect_signals(self) -> None:
        self.start_button.clicked.connect(self.start_requested.emit)

    def get_selected_mode(self) -> str:
        if self.all_wells_radio.isChecked():
            return "all"
        if self.selected_wells_radio.isChecked():
            return "selected"
        return "route"


    def get_volume_ml(self) -> float:
        return self.volume_spin.value()