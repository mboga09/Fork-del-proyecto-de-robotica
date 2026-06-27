from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
)


class ManualZPanel(QGroupBox):
    z_jog_requested = Signal(int, float)

    def __init__(self) -> None:
        super().__init__("Manual Z Jog")

        self.step_spin = QDoubleSpinBox()
        self.step_spin.setRange(0.5, 50.0)
        self.step_spin.setSingleStep(0.5)
        self.step_spin.setValue(5.0)
        self.step_spin.setSuffix(" mm")

        self.up_button = QPushButton("▲ Subir Z")
        self.down_button = QPushButton("▼ Bajar Z")

        self._build_ui()
        self._connect_signals()

    def _build_ui(self) -> None:
        layout = QVBoxLayout()
        buttons_layout = QHBoxLayout()

        buttons_layout.addWidget(self.up_button)
        buttons_layout.addWidget(self.down_button)

        layout.addWidget(QLabel("Paso manual:"))
        layout.addWidget(self.step_spin)
        layout.addLayout(buttons_layout)
        layout.addWidget(QLabel("Use pasos pequeños para verificar sentido de Z."))

        self.setLayout(layout)

    def _connect_signals(self) -> None:
        self.up_button.clicked.connect(lambda: self._emit_jog(direction=1))
        self.down_button.clicked.connect(lambda: self._emit_jog(direction=-1))

    def _emit_jog(self, direction: int) -> None:
        self.z_jog_requested.emit(direction, self.step_spin.value())

    def set_enabled(self, enabled: bool) -> None:
        self.up_button.setEnabled(enabled)
        self.down_button.setEnabled(enabled)
        self.step_spin.setEnabled(enabled)
