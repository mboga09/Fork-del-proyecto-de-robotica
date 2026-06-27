from __future__ import annotations

from typing import Any, Iterable

from control.serial_protocol import (
    Command,
    FIELD_COMMAND,
    FIELD_Z_DIR,
    FIELD_Z_TIME_S,
    make_home_command,
    make_stop_command,
    make_estop_command,
    make_move_act_command,
    make_tool_aspirate_command,
    make_tool_dispense_command,
)


class JsonMotionSender:
    """
    Adapta los comandos generados por MotionExecutor al protocolo JSON.

    En modo diagnóstico para ESP32 WROOM, cada comando de movimiento se envía
    y luego se espera explícitamente a que el firmware responda ACK y un estado
    terminal. Esto evita que Python sobreescriba acciones o sature el firmware
    enviando el siguiente punto antes de que el ESP32 reporte que terminó.
    """

    def __init__(
        self,
        serial_controller: Any,
        ack_timeout_s: float = 5.0,
        motion_timeout_s: float = 30.0,
    ):
        if not hasattr(serial_controller, "send_command"):
            raise TypeError(
                "serial_controller debe tener un método send_command(command: dict)."
            )

        self.serial = serial_controller
        self.ack_timeout_s = ack_timeout_s
        self.motion_timeout_s = motion_timeout_s

    # ---------------------------------------------------------
    # Helpers de envío con espera
    # ---------------------------------------------------------

    def _send_and_wait(
        self,
        command: dict,
        terminal_statuses: Iterable[str],
        timeout_s: float | None = None,
    ) -> dict:
        cmd = str(command.get("cmd", ""))
        timeout = self.motion_timeout_s if timeout_s is None else timeout_s

        self.serial.send_command(command)

        ack = self.serial.wait_for_ack(cmd, timeout_s=self.ack_timeout_s)
        if ack is None:
            raise TimeoutError(f"Timeout esperando ACK para {cmd}.")

        status = self.serial.wait_for_status(
            set(terminal_statuses),
            timeout_s=timeout,
        )
        if status is None:
            raise TimeoutError(
                f"Timeout esperando estado terminal {list(terminal_statuses)} para {cmd}."
            )

        status_name = status.get("status")
        if cmd not in ("STOP", "ESTOP") and status_name in ("STOPPED", "ESTOPPED"):
            raise RuntimeError(f"{cmd} interrumpido por estado {status_name}.")

        return status

    # ---------------------------------------------------------
    # Movimiento de actuadores
    # ---------------------------------------------------------

    def send_actuator_target(self, target) -> None:
        command = make_move_act_command(
            z_dir=target.z_direction,
            z_time_s=target.z_duration_s,
            s2_deg=target.servo2_deg,
            s3_deg=target.servo3_deg,
        )

        print("TX MOVE_ACT TARGET:", command, flush=True)
        self._send_and_wait(command, terminal_statuses=("IDLE", "STOPPED", "ESTOPPED"))

    def move_act(
        self,
        z_dir: int,
        z_time_s: float,
        s2_deg: float,
        s3_deg: float,
    ) -> None:
        """
        Helper para enviar MOVE_ACT directamente sin ActuatorTarget.
        Útil para pruebas manuales.
        """

        command = make_move_act_command(
            z_dir=z_dir,
            z_time_s=z_time_s,
            s2_deg=s2_deg,
            s3_deg=s3_deg,
        )
        self._send_and_wait(command, terminal_statuses=("IDLE", "STOPPED", "ESTOPPED"))

    def move_z_jog(self, z_dir: int, z_time_s: float) -> None:
        """
        Envía un jog puro de Z sin mandar s2_deg/s3_deg.

        El firmware conserva los servos rotativos en su último valor interno
        cuando esos campos no vienen en el JSON. Esto permite probar Z antes de
        HOME sin forzar q2/q3 a una pose asumida por Python.
        """

        z_dir = int(z_dir)
        z_time_s = float(z_time_s)

        if z_dir not in (-1, 1):
            raise ValueError("z_dir debe ser -1 o 1 para un jog de Z.")

        if z_time_s <= 0.0:
            raise ValueError("z_time_s debe ser positivo para un jog de Z.")

        command = {
            FIELD_COMMAND: Command.MOVE_ACT.value,
            FIELD_Z_DIR: z_dir,
            FIELD_Z_TIME_S: round(z_time_s, 3),
        }

        print("TX Z_JOG:", command, flush=True)
        self._send_and_wait(command, terminal_statuses=("IDLE", "STOPPED", "ESTOPPED"))

    # ---------------------------------------------------------
    # Comandos generales
    # ---------------------------------------------------------

    def home(self) -> None:
        self._send_and_wait(
            make_home_command(),
            terminal_statuses=("HOMED",),
            timeout_s=10.0,
        )

    def stop(self, wait: bool = True) -> None:
        command = make_stop_command()

        if not wait:
            self.serial.send_command(command)
            return

        self._send_and_wait(
            command,
            terminal_statuses=("STOPPED",),
            timeout_s=5.0,
        )

    def estop(self, wait: bool = True) -> None:
        command = make_estop_command()

        if not wait:
            self.serial.send_command(command)
            return

        self._send_and_wait(
            command,
            terminal_statuses=("ESTOPPED",),
            timeout_s=5.0,
        )

    # ---------------------------------------------------------
    # Herramienta
    # ---------------------------------------------------------

    def aspirate(self) -> None:
        self._send_and_wait(
            make_tool_aspirate_command(),
            terminal_statuses=("IDLE", "STOPPED", "ESTOPPED"),
        )

    def dispense(self) -> None:
        self._send_and_wait(
            make_tool_dispense_command(),
            terminal_statuses=("IDLE", "STOPPED", "ESTOPPED"),
        )
