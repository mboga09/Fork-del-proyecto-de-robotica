import json

import numpy as np
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
            z_up_speed_m_per_s=0.0012,
            z_down_speed_m_per_s=0.0012,
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
            initial_q=self.layout.q_z_calibration(),
            dry_run=True,
            command_sender=None,
            wait_after_send=True,
            servo_settle_s=0.05,
            status_callback=self.status_changed.emit,
        )

        # is_homed ahora significa que Z fue referenciado con el homing inicial.
        self.is_homed = False
        self.is_running = False

        self._thread: QThread | None = None
        self._worker: TransferWorker | None = None

    def connect_serial(self, port: str, baudrate: int) -> None:
        if self.serial_controller is not None and self.serial_controller.is_connected:
            self.status_changed.emit("Ya existe una conexión serial activa.")
            return

        self.serial_controller = SerialController(port=port, baud_rate=baudrate)
        self.serial_controller.on_message = self._on_firmware_message
        self.serial_controller.on_error = self._on_serial_error
        self.serial_controller.connect()

        if self.serial_controller.is_connected:
            self.motion_sender = JsonMotionSender(self.serial_controller)
            self.executor.dry_run = False
            self.executor.command_sender = self.motion_sender.send_actuator_target
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

    def initial_home(self) -> None:
        if not self._require_connection():
            return
        if self.is_running:
            self.status_changed.emit("No se puede hacer homing inicial mientras corre una tarea.")
            return

        self.status_changed.emit("Enviando homing inicial solo de Z.")
        try:
            self.motion_sender.initial_home()
            self._set_current_z_calibration_only()
            self.is_homed = True
            self.homed_changed.emit(True)
            self.status_changed.emit("Homing inicial completo. Solo d1 fue reiniciado a q_z_calibration.")
        except Exception as exc:
            self.is_homed = False
            self.homed_changed.emit(False)
            self.status_changed.emit(f"Error en homing inicial de Z: {exc}")

    def home(self) -> None:
        if not self._require_connection():
            return
        if self.is_running:
            self.status_changed.emit("No se puede ir a HOME mientras corre una tarea.")
            return
        if not self.is_homed:
            self.status_changed.emit("Error: primero debe ejecutar Initial Z Homing.")
            return

        q_home = self.layout.q_home()

        self.status_changed.emit("Moviendo a Route Home: primero Z, luego J2/J3.")
        try:
            self._move_z_first_to_q(
                q_goal=q_home,
                base_name="route_home",
            )

            self.status_changed.emit("Aplicando hold de Route Home en firmware.")
            self.motion_sender.home()
            self.executor.set_current_q(q_home)
            self.status_changed.emit("Route Home alcanzado y sostenido.")
        except Exception as exc:
            self.status_changed.emit(f"Error moviendo a Route Home: {exc}")

    def _move_z_first_to_q(self, q_goal, base_name: str) -> None:
        """
        Movimiento seguro hacia HOME/SAFE:
            1. Subir solo Z hasta el d1 objetivo.
            2. Con Z alto, mover J2/J3 hasta la configuracion objetivo.

        Esto evita barrer el workspace con el brazo mientras la herramienta
        todavia esta baja.
        """

        q_goal = np.asarray(q_goal, dtype=float)
        q_current = np.asarray(self.executor.current_q, dtype=float).copy()
        z_tolerance_m = 1e-6

        if q_current[0] < q_goal[0] - z_tolerance_m:
            q_raise = q_current.copy()
            q_raise[0] = q_goal[0]
            self.status_changed.emit(f"{base_name}: subiendo solo Z antes de mover J2/J3.")
            z_segment = self.planner.move_joint(
                q_start=q_current,
                q_goal=q_raise,
                steps=self.layout.linear_steps(),
                name=f"{base_name}_raise_z_first",
            )
            self.executor.execute_segment(z_segment)
            q_current = q_raise

        elif q_current[0] > q_goal[0] + z_tolerance_m:
            self.status_changed.emit(
                f"{base_name}: Z actual esta por encima del Z objetivo; no bajo Z para evitar colision."
            )
            q_goal = q_goal.copy()
            q_goal[0] = q_current[0]

        if np.allclose(q_current, q_goal, atol=1e-6):
            self.status_changed.emit(f"{base_name}: ya esta en configuracion objetivo.")
            return

        self.status_changed.emit(f"{base_name}: moviendo J2/J3 con Z alto.")
        arm_segment = self.planner.move_joint(
            q_start=q_current,
            q_goal=q_goal,
            steps=self.layout.joint_steps(),
            name=f"{base_name}_arm_at_safe_z",
        )
        self.executor.execute_segment(arm_segment)

    def stop(self) -> None:
        if self.motion_sender is not None:
            try:
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
        self.is_homed = False
        self.is_running = False
        self.homed_changed.emit(False)
        self.running_changed.emit(False)
        self.status_changed.emit("ESTOP enviado.")

    def manual_z_jog(self, direction: int, distance_mm: float) -> None:
        if not self._require_connection():
            return
        if self.is_running:
            self.status_changed.emit("No se puede hacer jog manual mientras corre una tarea.")
            return
        if not self.is_homed:
            self.status_changed.emit("Error: primero debe ejecutar Initial Z Homing.")
            return
        if direction not in (-1, 1):
            self.status_changed.emit(f"Dirección Z inválida: {direction}")
            return

        distance_m = float(distance_mm) / 1000.0
        if distance_m <= 0.0:
            self.status_changed.emit("La distancia de jog debe ser positiva.")
            return

        try:
            z_speed_m_per_s = self.mapper.z_speed_for_direction(direction)
            z_time_s = distance_m / z_speed_m_per_s
            self.status_changed.emit(
                f"Jog Z {'subir' if direction > 0 else 'bajar'}: "
                f"{distance_mm:.1f} mm, v={z_speed_m_per_s * 1000.0:.3f} mm/s, "
                f"t={z_time_s:.3f} s"
            )
            self.motion_sender.move_z_jog(z_dir=direction, z_time_s=z_time_s)
            self.executor.current_q[0] += direction * distance_m
            self.status_changed.emit(f"Jog Z terminado. d1 virtual={self.executor.current_q[0]:.4f} m")
        except Exception as exc:
            self.status_changed.emit(f"Error en jog Z: {exc}")

    def send_raw_serial_command(self, command_text: str) -> None:
        if self.serial_controller is None or not self.serial_controller.is_connected:
            self.status_changed.emit("Error: puerto serial no conectado.")
            return
        if self.is_running:
            self.status_changed.emit("No se puede enviar comando serial manual mientras corre una tarea.")
            return

        command_text = command_text.strip()
        if not command_text:
            self.status_changed.emit("Error: comando serial vacío.")
            return

        try:
            command = json.loads(command_text)
        except json.JSONDecodeError as exc:
            self.status_changed.emit(f"Error: JSON inválido: {exc}")
            return

        if not isinstance(command, dict):
            self.status_changed.emit("Error: el comando serial debe ser un objeto JSON.")
            return

        cmd = str(command.get("cmd", "UNKNOWN"))
        self.status_changed.emit(f"TX raw serial [{cmd}]: {command_text}")
        self.serial_controller.send_command(command)

    def start_transfer(self, wells: list[str]) -> None:
        if not self._require_connection():
            return
        if not self.is_homed:
            self.status_changed.emit("Error: debe ejecutar Initial Z Homing antes de iniciar.")
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

    def _on_firmware_message(self, message: dict) -> None:
        if not message:
            return
        print("RX FIRMWARE:", message, flush=True)
        self.status_changed.emit(get_message_text(message))

        status = message.get("status")
        homed = bool(message.get("homed", False))
        if status == "Z_HOMED":
            self.is_homed = True
            self._set_current_z_calibration_only()
            self.homed_changed.emit(True)
            self.status_changed.emit("Z calibrado. Solo d1 fue reiniciado a q_z_calibration.")
            return
        if status == "HOMED":
            self.executor.set_current_q(self.layout.q_home())
            self.status_changed.emit("Route Home confirmado por firmware.")
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

    def _set_current_z_calibration_only(self) -> None:
        q_current = np.asarray(self.executor.current_q, dtype=float).copy()
        q_current[0] = float(self.layout.q_z_calibration()[0])
        self.executor.set_current_q(q_current)

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
