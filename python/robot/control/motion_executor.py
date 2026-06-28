from typing import Callable, Iterable, List, Optional
import time

import numpy as np
from spatialmath import SE3

from robot.control.actuator_mapping import ActuatorMapper, ActuatorTarget
from robot.trajectory.path_planner import MotionSegment


class MotionExecutor:
    """
    Convierte trayectorias de alto nivel en comandos discretos de actuador.

    Entrada:
        MotionSegment generado por PathPlanner.

    Salida:
        ActuatorTarget generado por ActuatorMapper.

    En modo dry_run:
        imprime los comandos sin enviarlos al STM32.

    En modo real:
        usa command_sender para mandar comandos seriales.
        Normalmente command_sender será:

            JsonMotionSender.send_actuator_target
    """

    def __init__(
        self,
        robot_model,
        actuator_mapper: ActuatorMapper,
        initial_q,
        dry_run: bool = True,
        command_sender: Optional[Callable[[ActuatorTarget], None]] = None,
        wait_after_send: bool = True,
        servo_settle_s: float = 0.05,
        skip_duplicate_points: bool = True,
        status_callback: Optional[Callable[[str], None]] = None,
    ):
        self.robot = robot_model
        self.mapper = actuator_mapper
        self.current_q = np.asarray(initial_q, dtype=float)

        if self.current_q.shape != (3,):
            raise ValueError("initial_q debe tener forma [d1, theta2, theta3].")

        self.dry_run = dry_run
        self.command_sender = command_sender

        self.wait_after_send = wait_after_send
        self.servo_settle_s = servo_settle_s
        self.skip_duplicate_points = skip_duplicate_points
        self.status_callback = status_callback

        self._validate_joint_limits(self.current_q)

    # ---------------------------------------------------------
    # API principal
    # ---------------------------------------------------------

    def execute_path(self, path: List[MotionSegment]) -> None:
        for segment in path:
            self.execute_segment(segment)

    def execute_segment(self, segment: MotionSegment) -> None:
        self._emit_status(f"Ejecutando segmento: {segment.name} ({segment.type})")

        if segment.type == "joint":
            q_path = self._joint_trajectory_to_q_list(segment.trajectory)

        elif segment.type == "cartesian":
            q_path = self._cartesian_trajectory_to_q_list(segment.trajectory)

        else:
            raise ValueError(f"Tipo de segmento desconocido: {segment.type}")

        for index, q_target in enumerate(q_path):
            self._execute_joint_point(q_target, index=index)

    def set_current_q(self, q) -> None:
        """
        Actualiza la configuración articular interna del executor.

        Se usa después de HOME, porque el firmware lleva físicamente
        el robot a una referencia conocida.
        """

        q = np.asarray(q, dtype=float)

        if q.shape != (3,):
            raise ValueError("q debe tener forma [d1, theta2, theta3].")

        self._validate_joint_limits(q)
        self.current_q = q.copy()

    # ---------------------------------------------------------
    # Conversión de trayectorias
    # ---------------------------------------------------------

    def _joint_trajectory_to_q_list(self, trajectory) -> List[np.ndarray]:
        """
        Convierte la salida de jtraj/free_joint a lista de q.
        """

        if not hasattr(trajectory, "q"):
            raise ValueError("La trayectoria articular no tiene atributo .q")

        return [
            np.asarray(q, dtype=float)
            for q in trajectory.q
        ]

    def _cartesian_trajectory_to_q_list(
        self,
        trajectory: Iterable[SE3],
    ) -> List[np.ndarray]:
        """
        Convierte una trayectoria cartesiana SE3[] en trayectoria articular
        usando cinemática inversa.

        Como el robot solo controla posición y la orientación es constante,
        usamos mask=[1, 1, 1, 0, 0, 0].

        joint_limits=False evita que Robotics Toolbox evalúe el eje Z con
        qlim infinito. Después se validan solo las articulaciones rotacionales.
        """

        q_list = []
        q_seed = self.current_q.copy()

        for i, T in enumerate(trajectory):
            sol = self.robot.ikine_LM(
                T,
                q0=q_seed,
                mask=[1, 1, 1, 0, 0, 0],
                joint_limits=False,
            )

            if not sol.success:
                raise RuntimeError(
                    f"IK falló en punto cartesiano {i}. "
                    f"Pose:\n{T}\n"
                    f"Mensaje: {sol.reason}"
                )

            q = np.asarray(sol.q, dtype=float)
            self._validate_joint_limits(q)

            q_list.append(q)
            q_seed = q

        return q_list

    # ---------------------------------------------------------
    # Ejecución de punto
    # ---------------------------------------------------------

    def _execute_joint_point(self, q_target, index: int = 0) -> None:
        q_target = np.asarray(q_target, dtype=float)

        if q_target.shape != (3,):
            raise ValueError("q_target debe tener forma [d1, theta2, theta3].")

        self._validate_joint_limits(q_target)

        if self.skip_duplicate_points and self._is_same_q(q_target, self.current_q):
            return

        actuator_target = self.mapper.joint_to_actuator(
            q_target=q_target,
            q_current=self.current_q,
        )

        if self.dry_run:
            self._print_command(index, q_target, actuator_target)

        else:
            if self.command_sender is None:
                raise RuntimeError(
                    "dry_run=False pero no se proporcionó command_sender."
                )

            self.command_sender(actuator_target)

            if self.wait_after_send:
                self._wait_for_physical_motion(actuator_target)

        self.current_q = q_target.copy()

    # ---------------------------------------------------------
    # Espera física mínima
    # ---------------------------------------------------------

    def _wait_for_physical_motion(self, actuator_target: ActuatorTarget) -> None:
        """
        Evita saturar el firmware enviando todos los puntos inmediatamente.

        Como el eje Z usa servo continuo, MOVE_ACT contiene z_duration_s.
        Python espera ese tiempo antes de enviar el siguiente punto.
        """

        wait_s = max(
            float(actuator_target.z_duration_s),
            float(self.servo_settle_s),
        )

        if wait_s > 0.0:
            time.sleep(wait_s)

    # ---------------------------------------------------------
    # Validación
    # ---------------------------------------------------------

    def _validate_joint_limits(self, q) -> None:
        """
        Valida solo las articulaciones rotacionales q2/q3.

        El eje Z d1 usa finales de carrera físicos y no debe entrar en la
        validación de joint limits del modelo porque su qlim es infinito.
        """
        q = np.asarray(q, dtype=float)

        q_min = self.robot.qlim[0]
        q_max = self.robot.qlim[1]

        tolerance = 1e-6
        rot_slice = slice(1, 3)

        if (
            np.any(q[rot_slice] < q_min[rot_slice] - tolerance)
            or np.any(q[rot_slice] > q_max[rot_slice] + tolerance)
        ):
            raise ValueError(
                "Configuración rotacional fuera de límites:\n"
                f"q     = {q}\n"
                f"q_min = {q_min}\n"
                f"q_max = {q_max}"
            )

    @staticmethod
    def _is_same_q(q_a, q_b, tolerance: float = 1e-7) -> bool:
        return np.allclose(q_a, q_b, atol=tolerance, rtol=0.0)

    # ---------------------------------------------------------
    # Estado / debug
    # ---------------------------------------------------------

    def _emit_status(self, message: str) -> None:
        if self.status_callback is not None:
            self.status_callback(message)
        else:
            print(f"\n=== {message} ===")

    @staticmethod
    def _print_command(
        index: int,
        q_target,
        actuator_target: ActuatorTarget,
    ) -> None:
        d1 = q_target[0]
        q2_deg = np.rad2deg(q_target[1])
        q3_deg = np.rad2deg(q_target[2])

        print(
            f"[{index:03d}] "
            f"q=[d1={d1:.4f} m, q2={q2_deg:.2f} deg, q3={q3_deg:.2f} deg] "
            f"-> "
            f"ZDIR={actuator_target.z_direction}, "
            f"ZTIME={actuator_target.z_duration_s:.3f} s, "
            f"S2={actuator_target.servo2_deg:.2f} deg, "
            f"S3={actuator_target.servo3_deg:.2f} deg"
        )
