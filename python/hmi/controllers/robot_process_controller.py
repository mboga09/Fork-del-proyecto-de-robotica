from PySide6.QtCore import QObject, Signal, QThread

from control.serial_controller import SerialController
from control.serial_protocol import get_message_text

from robot.model.model import ScaraPRR
from robot.workspace.lab_layout import LabLayout
from robot.trajectory.path_planner import PathPlanner
from robot.control.actuator_mapping import ActuatorMapper
from robot.control.motion_executor import MotionExecutor
from robot.control.json_motion_sender import JsonMotionSender
from robot.tasks.liquid_transfer import LiquidTransferTask


class TransferWorker(QObject):
    finished = Signal()
    error = Signal(str)
    status = Signal(str)

    def __init__(self, task: LiquidTransferTask, wells: list[str]):
        super().__init__()
        self.task = task
        self.wells = wells

    def run(self) -> None:
        try:
            self.task.run_wells(self.wells)
        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            self.finished.emit()


class RobotProcessController(QObject):
    status_changed = Signal(str)
    connection_changed = Signal(bool)
    homed_changed = Signal(bool)
    running_changed = Signal(bool)

    def __init__(self):
        super().__init__()

        self.serial_controller: SerialController | None = None
        self.motion_sender: JsonMotionSender | None = None

        self.robot = ScaraPRR()
        self.layout = LabLayout()

        self.mapper = ActuatorMapper(
            z_pitch_m_per_rev=0.002,
            z_speed_m_per_s=0.0060,
            z_min_m=None,
            z_max_m=None,
        )

        self.planner = PathPlanner(
            robot_model=self.robot,
            safe_pose=self.robot.fkine(self.layout.q_safe()),
            source_pose=self.layout.source_pose(),
            approach_height=self.layout.approach_height_m(),
        )

        self.executor = MotionExecutor(
            robot_model=self.robot,
            actuator_mapper=self.mapper,
            initial_q=self.layout.q_home(),
            dry_run=True,
            command_sender=None,
            wait_after_send=True,
            servo_settle_s=0.05,
            status_callback=self.status_changed.emit,
        )

        self.is_homed = False
        self.is_running = False

        self._thread: QThread | None = None
        self._worker: TransferWorker | None = None

    # ---------------------------------------------------------
    # Serial
    # ---------------------------------------------------------

    def connect_serial(self, port: str, baudrate: int) -> None:
        if self.serial_controller is not None and self.serial_controller.is_connected:
            self.status_changed.emit("Ya existe una conexión serial activa.")
            return

        self.serial_controller = SerialController(
            port=port,
            baud_rate=baudrate,
        )

        self.serial_controller.on_message = self._on_firmware_message
        self.serial_controller.on_error = self._on_serial_error

        self.serial_controller.connect()

        if self.serial_controller.is_connected:
            self.motion_sender = JsonMotionSender(self.serial_controller)

            self.executor.dry_run = False
            self.executor.command_sender = self.motion_sender.send_actuator_target
            # En este diagnóstico el JsonMotionSender ya espera ACK + estado
            # terminal del ESP32. No hacemos un segundo sleep en Python.
            self.executor.wait_after_send = False

            self.connection_changed.emit(True)
            self.status_changed.emit(f"Conectado a {port} @ {baudrate}.")
        else:
            self.connection_changed.emit(False)
            self.status_changed.emit("No se pudo abrir el puerto serial.")

    def disconnect_serial(self) -> None:
        if self.serial_controller is not None:
            self.serial_controller.disconnect()

        self.motion_sender = None
        self.executor.dry_run = True
        self.executor.command_sender = None
        self.executor.wait_after_send = True

        self.connection_changed.emit(False)
        self.status_changed.emit("Desconectado.")

    # ---------------------------------------------------------
    # Comandos directos
    # ---------------------------------------------------------

    def home(self) -> None:
        if not self._require_connection():
            return

        self.status_changed.emit("Enviando HOME.")

        try:
            self.motion_sender.home()
        except Exception as exc:
            self.status_changed.emit(f"Error enviando HOME: {exc}")

    def stop(self) -> None:
        if self.motion_sender is not None:
            try:
                # STOP es la excepción de seguridad: se envía inmediatamente sin
                # bloquear la HMI esperando respuesta. El hilo de lectura seguirá
                # reportando ACK/STOPPED cuando el ESP32 los mande.
                self.motion_sender.stop(wait=False)
            except Exception as exc:
                self.status_changed.emit(f"Error enviando STOP: {exc}")

        self.is_running = False
        self.running_changed.emit(False)
        self.status_changed.emit("STOP enviado.")

    def estop(self) -> None:
        if self.motion_sender is not None:
            try:
                self.motion_sender.estop(wait=False)
            except Exception as exc:
                self.status_changed.emit(f"Error enviando ESTOP: {exc}")

        self.is_running = False
        self.running_changed.emit(False)
        self.status_changed.emit("ESTOP enviado.")

    def manual_z_jog(self, direction: int, distance_mm: float) -> None:
        if not self._require_connection():
            return

        if self.is_running:
            self.status_changed.emit("No se puede hacer jog manual mientras corre una tarea.")
            return

        if direction not in (-1, 1):
            self.status_changed.emit(f"Dirección Z inválida: {direction}")
            return

        distance_m = float(distance_mm) / 1000.0
        if distance_m <= 0.0:
            self.status_changed.emit("La distancia de jog debe ser positiva.")
            return

        try:
            z_time_s = distance_m / self.mapper.z_speed_m_per_s
            self.status_changed.emit(
                f"Jog Z {'subir' if direction > 0 else 'bajar'}: "
                f"{distance_mm:.1f} mm, t={z_time_s:.3f} s"
            )
            self.motion_sender.move_z_jog(z_dir=direction, z_time_s=z_time_s)
            self.executor.current_q[0] += direction * distance_m
            self.status_changed.emit(
                "Jog Z terminado. "
                f"d1 virtual={self.executor.current_q[0]:.4f} m"
            )
        except Exception as exc:
            self.status_changed.emit(f"Error en jog Z: {exc}")

    # ---------------------------------------------------------
    # Transferencia
    # ---------------------------------------------------------

    def start_transfer(self, wells: list[str]) -> None:
        if not self._require_connection():
            return

        if not self.is_homed:
            self.status_changed.emit("Error: debe hacer HOME antes de iniciar.")
            return
        if self.is_running:
            self.status_changed.emit("Ya hay una tarea corriendo.")
            return
        if not wells:
            self.status_changed.emit("Error: no hay wells seleccionados.")
            return

        task = LiquidTransferTask(
            planner=self.planner,
            executor=self.executor,
            motion_sender=self.motion_sender,
            layout=self.layout,
            status_callback=self.status_changed.emit,
        )

        self._thread = QThread()
        self._worker = TransferWorker(task, wells)

        self._worker.moveToThread(self._thread)

        self._thread.started.connect(self._worker.run)
        self._worker.status.connect(self.status_changed.emit)
        self._worker.error.connect(self._on_worker_error)
        self._worker.finished.connect(self._on_worker_finished)

        self._worker.finished.connect(self._thread.quit)
        self._worker.finished.connect(self._worker.deleteLater)
        self._thread.finished.connect(self._thread.deleteLater)

        self.is_running = True
        self.running_changed.emit(True)

        self.status_changed.emit(f"Iniciando tarea para wells: {wells}")
        self._thread.start()

    # ---------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------

    def _on_firmware_message(self, message: dict) -> None:
        if not message:
            return

        print("RX FIRMWARE:", message, flush=True)

        self.status_changed.emit(get_message_text(message))

        status = message.get("status")
        homed = bool(message.get("homed", False))

        if status == "HOMED":
            self.is_homed = True
            self.executor.set_current_q(self.layout.q_home())
            self.homed_changed.emit(True)
            self.status_changed.emit("Robot homed. current_q reiniciado a q_home.")
            return

        if homed:
            self.is_homed = True
            self.homed_changed.emit(True)

        if status == "HOMING":
            self.is_homed = False
            self.homed_changed.emit(False)
            return

        if status == "STOPPED":
            self.is_running = False
            self.running_changed.emit(False)
            return

        if status == "ESTOPPED":
            self.is_homed = False
            self.is_running = False
            self.homed_changed.emit(False)
            self.running_changed.emit(False)
            return

    def _on_serial_error(self, message: str) -> None:
        self.status_changed.emit(f"Serial error: {message}")

    def _on_worker_error(self, message: str) -> None:
        self.status_changed.emit(f"Task error: {message}")

    def _on_worker_finished(self) -> None:
        self.is_running = False
        self.running_changed.emit(False)
        self.status_changed.emit("Tarea terminada.")

    def _require_connection(self) -> bool:
        if self.serial_controller is None or not self.serial_controller.is_connected:
            self.status_changed.emit("Error: puerto serial no conectado.")
            return False

        if self.motion_sender is None:
            self.status_changed.emit("Error: motion sender no inicializado.")
            return False

        return True
