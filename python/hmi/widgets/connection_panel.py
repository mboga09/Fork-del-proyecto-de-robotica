from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QComboBox,
    QPushButton,
)


class ConnectionPanel(QGroupBox):
    connect_requested = Signal(str, int)
    disconnect_requested = Signal()

    def __init__(self, serial_config: dict | None = None) -> None:
        super().__init__("Connection")

        self.serial_config = serial_config or {}

        self.port_combo = QComboBox()
        self.baudrate_combo = QComboBox()

        self._load_config_values()

        self.connect_button = QPushButton("Connect")
        self.disconnect_button = QPushButton("Disconnect")

        self.status_label = QLabel("Status: Disconnected")

        self._build_ui()
        self._connect_signals()

    def _load_config_values(self) -> None:
        ports = self.serial_config.get(
            "available_ports",
            ["COM3", "COM4", "/dev/ttyUSB0", "/dev/ttyACM0"],
        )

        baudrates = self.serial_config.get(
            "standard_baudrates",
            [9600, 19200, 38400, 57600, 115200],
        )

        default_port = self.serial_config.get("default_port", ports[0])
        default_baudrate = self.serial_config.get("default_baudrate", 115200)

        self.port_combo.addItems([str(port) for port in ports])
        self.baudrate_combo.addItems([str(baudrate) for baudrate in baudrates])

        if str(default_port) in [self.port_combo.itemText(i) for i in range(self.port_combo.count())]:
            self.port_combo.setCurrentText(str(default_port))

        if str(default_baudrate) in [self.baudrate_combo.itemText(i) for i in range(self.baudrate_combo.count())]:
            self.baudrate_combo.setCurrentText(str(default_baudrate))



    def _build_ui(self) -> None:
        layout = QVBoxLayout()

        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Port:"))
        port_layout.addWidget(self.port_combo)

        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baudrate:"))
        baud_layout.addWidget(self.baudrate_combo)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.connect_button)
        button_layout.addWidget(self.disconnect_button)

        layout.addLayout(port_layout)
        layout.addLayout(baud_layout)
        layout.addLayout(button_layout)
        layout.addWidget(self.status_label)

        self.setLayout(layout)

    def _connect_signals(self) -> None:
        self.connect_button.clicked.connect(self._emit_connect)
        self.disconnect_button.clicked.connect(self.disconnect_requested.emit)

    def _emit_connect(self) -> None:
        port = self.port_combo.currentText()
        baudrate = int(self.baudrate_combo.currentText())
        self.connect_requested.emit(port, baudrate)