from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class ActuatorTarget:
    z_delta_m: float
    z_revolutions: float
    z_direction: int
    z_duration_s: float

    servo2_deg: float
    servo3_deg: float


class ActuatorMapper:
    """
    Mapper para:

    J1: MG996R continuo + tornillo 2 mm/rev
    J2: MG996R 180 deg, con limite cinematico actual [-30 deg, 30 deg]
    J3: MG996R 180 deg, con limite cinematico actual [-45 deg, 45 deg]
    """

    def __init__(
        self,
        z_pitch_m_per_rev: float = 0.002,

        # Velocidad efectiva calibrada para el eje Z.
        # Valor anterior: 0.0060 m/s.
        # Se duplica para reducir a la mitad los tiempos enviados.
        z_speed_m_per_s: float = 0.0120,

        # El eje Z usa finales de carrera fisicos. Por defecto no se limita
        # por software en el mapper para permitir jog manual antes de HOME.
        z_min_m: Optional[float] = None,
        z_max_m: Optional[float] = None,

        # J2:
        # q2 = 0 deg -> servo = 45 deg
        # q2 positivo aumenta el angulo enviado al servo.
        q2_servo_at_zero_deg: float = 45.0,
        q2_ratio: float = 1.0,
        q2_direction: float = 1.0,

        # J3:
        # El servo esta montado al reves y usa una escala 2:1 respecto al
        # angulo q3 del modelo.
        # q3 = -45 deg -> senal = 180 deg
        # q3 =   0 deg -> senal = 90 deg
        # q3 =  45 deg -> senal = 0 deg
        q3_servo_at_zero_deg: float = 90.0,
        q3_ratio: float = 2.0,
        q3_direction: float = -1.0,

        servo_min_deg: float = 0.0,
        servo_max_deg: float = 180.0,
    ):
        self.z_pitch_m_per_rev = z_pitch_m_per_rev
        self.z_speed_m_per_s = z_speed_m_per_s
        self.z_min_m = z_min_m
        self.z_max_m = z_max_m

        self.q2_servo_at_zero_deg = q2_servo_at_zero_deg
        self.q2_ratio = q2_ratio
        self.q2_direction = q2_direction

        self.q3_servo_at_zero_deg = q3_servo_at_zero_deg
        self.q3_ratio = q3_ratio
        self.q3_direction = q3_direction

        self.servo_min_deg = servo_min_deg
        self.servo_max_deg = servo_max_deg

    def joint_to_actuator(self, q_target, q_current) -> ActuatorTarget:
        q_target = np.asarray(q_target, dtype=float)
        q_current = np.asarray(q_current, dtype=float)

        if q_target.shape != (3,):
            raise ValueError("q_target debe ser [d1, theta2, theta3].")

        if q_current.shape != (3,):
            raise ValueError("q_current debe ser [d1, theta2, theta3].")

        d1_target_m = q_target[0]
        d1_current_m = q_current[0]

        theta2_deg = np.rad2deg(q_target[1])
        theta3_deg = np.rad2deg(q_target[2])

        self._validate_d1(d1_target_m)

        z_delta_m = d1_target_m - d1_current_m

        if abs(z_delta_m) < 1e-6:
            z_direction = 0
            z_revolutions = 0.0
            z_duration_s = 0.0
        else:
            z_direction = 1 if z_delta_m > 0 else -1
            z_revolutions = abs(z_delta_m) / self.z_pitch_m_per_rev
            z_duration_s = abs(z_delta_m) / self.z_speed_m_per_s

        servo2_deg = (
            self.q2_servo_at_zero_deg
            + self.q2_direction * self.q2_ratio * theta2_deg
        )

        servo3_deg = (
            self.q3_servo_at_zero_deg
            + self.q3_direction * self.q3_ratio * theta3_deg
        )

        self._validate_servo("servo2", servo2_deg)
        self._validate_servo("servo3", servo3_deg)

        return ActuatorTarget(
            z_delta_m=z_delta_m,
            z_revolutions=z_revolutions,
            z_direction=z_direction,
            z_duration_s=z_duration_s,
            servo2_deg=servo2_deg,
            servo3_deg=servo3_deg,
        )

    def _validate_d1(self, d1_m: float):
        if self.z_min_m is not None and d1_m < self.z_min_m:
            raise ValueError(
                f"d1={d1_m:.4f} m esta por debajo del minimo {self.z_min_m:.4f} m."
            )

        if self.z_max_m is not None and d1_m > self.z_max_m:
            raise ValueError(
                f"d1={d1_m:.4f} m esta por encima del maximo {self.z_max_m:.4f} m."
            )

    def _validate_servo(self, name: str, angle_deg: float):
        if angle_deg < self.servo_min_deg or angle_deg > self.servo_max_deg:
            raise ValueError(
                f"{name}={angle_deg:.2f} deg fuera de rango "
                f"[{self.servo_min_deg}, {self.servo_max_deg}] deg."
            )
