from __future__ import annotations

from typing import Any

from control.serial_protocol import (
    make_home_command,
    make_stop_command,
    make_estop_command,
    make_move_act_command,
    make_tool_aspirate_command,
    make_tool_dispense_command,
)


class JsonMotionSender:
    """
    Adapta los comandos generados por MotionExecutor al protocolo JSON
    existente del proyecto.

    Recibe ActuatorTarget y lo convierte a:

        {
            "cmd": "MOVE_ACT",
            "z_dir": ...,
            "z_time_s": ...,
            "s2_deg": ...,
            "s3_deg": ...
        }

    No calcula cinemática.
    No genera trayectorias.
    No conoce la HMI.

    Solo convierte comandos de movimiento a JSON y los envía usando
    SerialController.send_command(...).
    """

    def __init__(self, serial_controller: Any):
        """
        Parameters
        ----------
        serial_controller:
            Instancia de control.serial_controller.SerialController,
            o cualquier objeto que tenga un método send_command(dict).
        """

        if not hasattr(serial_controller, "send_command"):
            raise TypeError(
                "serial_controller debe tener un método send_command(command: dict)."
            )

        self.serial = serial_controller

    # ---------------------------------------------------------
    # Movimiento de actuadores
    # ---------------------------------------------------------
    
    def send_actuator_target(self, target) -> None:
        """
        Envía un ActuatorTarget como comando JSON MOVE_ACT.
        """

        command = make_move_act_command(
            z_dir=target.z_direction,
            z_time_s=target.z_duration_s,
            s2_deg=target.servo2_deg,
            s3_deg=target.servo3_deg,
        )

        self.serial.send_command(command)

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

        self.serial.send_command(command)

    # ---------------------------------------------------------
    # Comandos generales
    # ---------------------------------------------------------

    def home(self) -> None:
        self.serial.send_command(make_home_command())

    def stop(self) -> None:
        self.serial.send_command(make_stop_command())

    def estop(self) -> None:
        self.serial.send_command(make_estop_command())

    # ---------------------------------------------------------
    # Herramienta
    # ---------------------------------------------------------

    def aspirate(self) -> None:
        self.serial.send_command(make_tool_aspirate_command())

    def dispense(self) -> None:
        self.serial.send_command(make_tool_dispense_command())

    def send_actuator_target(self, target) -> None:
        command = make_move_act_command(
        z_dir=target.z_direction,
        z_time_s=target.z_duration_s,
        s2_deg=target.servo2_deg,
        s3_deg=target.servo3_deg,
    )

        print("TX MOVE_ACT:", command, flush=True)

        self.serial.send_command(command)