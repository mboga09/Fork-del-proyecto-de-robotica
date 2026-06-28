from PySide6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
)

from tools.config_loader import load_yaml_config

from hmi.widgets.connection_panel import ConnectionPanel
from hmi.widgets.homing_panel import HomingPanel
from hmi.widgets.operation_panel import OperationPanel
from hmi.widgets.well_selector import WellSelector
from hmi.widgets.status_panel import StatusPanel
from hmi.widgets.manual_z_panel import ManualZPanel
from hmi.widgets.raw_serial_panel import RawSerialPanel

from hmi.controllers.robot_process_controller import RobotProcessController


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle("Robotic Pipetting HMI")
        self.setMinimumSize(1100, 700)

        serial_config_file = load_yaml_config("serial_config.yaml")
        serial_config = serial_config_file.get("serial", {})

        plate_config_file = load_yaml_config("plate_config.yaml")
        plate_config = plate_config_file.get("plate", {})

        self.connection_panel = ConnectionPanel(serial_config=serial_config)
        self.homing_panel = HomingPanel()
        self.manual_z_panel = ManualZPanel()
        self.raw_serial_panel = RawSerialPanel()
        self.operation_panel = OperationPanel()
        self.well_selector = WellSelector(plate_config=plate_config)
        self.status_panel = StatusPanel()

        self.robot_controller = RobotProcessController()
        self._serial_connected = False

        self.raw_serial_panel.set_enabled(False)

        self._build_ui()
        self._connect_signals()

    def _build_ui(self) -> None:
        central_widget = QWidget()
        main_layout = QVBoxLayout()

        title = QLabel("Robotic Pipetting HMI")
        title.setObjectName("titleLabel")

        top_layout = QHBoxLayout()
        top_layout.addWidget(self.connection_panel)
        top_layout.addWidget(self.homing_panel)
        top_layout.addWidget(self.manual_z_panel)

        middle_layout = QHBoxLayout()
        middle_layout.addWidget(self.operation_panel)
        middle_layout.addWidget(self.well_selector)

        main_layout.addWidget(title)
        main_layout.addLayout(top_layout)
        main_layout.addWidget(self.raw_serial_panel)
        main_layout.addLayout(middle_layout)
        main_layout.addWidget(self.status_panel)

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def _connect_signals(self) -> None:
        self.connection_panel.connect_requested.connect(self._on_connect_requested)
        self.connection_panel.disconnect_requested.connect(self.robot_controller.disconnect_serial)

        self.homing_panel.initial_home_requested.connect(self.robot_controller.initial_home)
        self.homing_panel.home_requested.connect(self.robot_controller.home)
        self.homing_panel.stop_requested.connect(self.robot_controller.stop)
        self.homing_panel.estop_requested.connect(self.robot_controller.estop)

        self.manual_z_panel.z_jog_requested.connect(self.robot_controller.manual_z_jog)
        self.raw_serial_panel.command_requested.connect(self._on_raw_serial_command_requested)

        self.operation_panel.start_requested.connect(self._on_start_requested)
        self.well_selector.well_selection_changed.connect(self._on_well_selection_changed)

        self.robot_controller.status_changed.connect(self.status_panel.log_message)
        self.robot_controller.connection_changed.connect(self._on_connection_changed)
        self.robot_controller.homed_changed.connect(self._on_homed_changed)
        self.robot_controller.running_changed.connect(self._on_running_changed)

    # ---------------------------------------------------------
    # HMI callbacks
    # ---------------------------------------------------------

    def _on_connect_requested(self, port: str, baudrate: int) -> None:
        self.status_panel.log_message(f"Conectando a {port} @ {baudrate}")
        self.robot_controller.connect_serial(port, baudrate)

    def _on_start_requested(self) -> None:
        mode = self.operation_panel.get_selected_mode()

        if mode == "all":
            wells = self.well_selector.get_all_wells()

        elif mode == "selected":
            wells = self.well_selector.get_route_wells()

        else:
            self.status_panel.log_message(f"Error: modo desconocido: {mode}")
            return

        self.status_panel.log_message(f"Start solicitado. Modo: {mode}. Wells: {wells}")
        self.robot_controller.start_transfer(wells)

    def _on_well_selection_changed(self, wells: list[str]) -> None:
        ordered_wells = self.well_selector.get_route_wells()
        self.status_panel.log_message(
            f"Wells seleccionados: {wells}. Orden de ruta: {ordered_wells}"
        )

    def _on_raw_serial_command_requested(self, command_text: str) -> None:
        self.robot_controller.send_raw_serial_command(command_text)

    # ---------------------------------------------------------
    # Robot controller callbacks
    # ---------------------------------------------------------

    def _on_connection_changed(self, connected: bool) -> None:
        self._serial_connected = connected
        self.raw_serial_panel.set_enabled(connected and not self.robot_controller.is_running)

        if connected:
            self.connection_panel.status_label.setText("Status: Connected")
        else:
            self.connection_panel.status_label.setText("Status: Disconnected")
            self.homing_panel.homed_label.setText("Z Calibrated: No")
            self.homing_panel.state_label.setText("State: Disconnected")

    def _on_homed_changed(self, homed: bool) -> None:
        self.homing_panel.homed_label.setText("Z Calibrated: Yes" if homed else "Z Calibrated: No")

    def _on_running_changed(self, running: bool) -> None:
        self.homing_panel.state_label.setText("State: Running" if running else "State: Idle")
        self.operation_panel.start_button.setEnabled(not running)
        self.manual_z_panel.set_enabled(not running)
        self.raw_serial_panel.set_enabled(self._serial_connected and not running)
