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
    Mapper para convertir el espacio articular q=[d1, theta2, theta3]
    al espacio de actuadores [z, servo2, servo3].

    Calibracion actual:
        q2 = 0 deg -> servo2 = 90 deg.
        q3 = 55 deg -> servo3 = 180 deg.

    Conversion inversa para teach points:
        theta2_deg = servo2_deg - 90
        theta3_deg = 235 - servo3_deg
    """

    def __init__(
        self,
        z_pitch_m_per_rev: float = 0.002,

        # Velocidad efectiva calibrada para el eje Z.
        # Si un comando de 20 mm movia aproximadamente 2 mm,
        # el tiempo enviado debe ser 10x mayor: 0.0120 -> 0.0012 m/s.
        z_speed_m_per_s: float = 0.0012,

        # El eje Z usa finales de carrera fisicos. Por defecto no se limita
        # por software en el mapper para permitir jog manual antes de HOME.
        z_min_m: Optional[float] = None,
        z_max_m: Optional[float] = None,

        # J2:
        # q2 = 0 deg -> servo2 = 90 deg.
        # q2 positivo aumenta el angulo enviado al servo.
        q2_servo_at_zero_deg: float = 90.0,
        q2_ratio: float = 1.0,
        q2_direction: float = 1.0,

        # J3:
        # Se calcula primero el angulo logico 1:1 del servo:
        #   q3_logico = q3_servo_at_zero_deg + q3_ratio * theta3_deg
        # Luego, como el servo esta montado al reves, se invierte la senal:
        #   servo3 = 180 - q3_logico
        # Para q3 = 55 deg, q3_logico = 0 deg y servo3 = 180 deg.
        q3_servo_at_zero_deg: float = -55.0,
        q3_ratio: float = 1.0,
        q3_direction: float = 1.0,

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

        servo3_logical_deg = (
            self.q3_servo_at_zero_deg
            + self.q3_direction * self.q3_ratio * theta3_deg
        )
        servo3_deg = self.servo_max_deg - servo3_logical_deg

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
