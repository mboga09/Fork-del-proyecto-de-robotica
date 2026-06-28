from dataclasses import dataclass
from typing import Any, List, Optional

import numpy as np
from spatialmath import SE3

# Intentamos mantener compatibilidad con ambas arquitecturas:
# 1. robot.trajectory.generators
# 2. robot.trajectory.point_to_point / linear_cartesian
try:
    from robot.trajectory.generators import (
        free_joint,
        linear_cartesian,
    )
except ImportError:
    from robot.trajectory.point_to_point import free_joint
    from robot.trajectory.linear_cartesian import linear_cartesian


@dataclass
class MotionSegment:
    """
    Segmento elemental de movimiento.

    type:
        - "joint": trayectoria articular generada con jtraj()
        - "cartesian": trayectoria cartesiana generada con ctraj()

    name:
        Nombre descriptivo del segmento. Sirve para que la capa de tarea
        sepa dónde insertar acciones como aspirar o dispensar.
    """

    type: str
    trajectory: Any
    name: str = ""


class PathPlanner:
    """
    Planificador de trayectorias de alto nivel.

    Equivalencias:
        ABB MoveJ -> free_joint()       -> jtraj()
        ABB MoveL -> linear_cartesian() -> ctraj()

    Esta clase NO ejecuta movimientos.
    Esta clase NO envía comandos seriales.
    Esta clase NO controla la herramienta.

    Solo construye una lista ordenada de segmentos de movimiento.
    """

    def __init__(
        self,
        robot_model,
        safe_pose: SE3,
        source_pose: SE3,
        approach_height: float = 0.025,
    ):
        """
        Parameters
        ----------
        robot_model:
            Modelo del robot.

        safe_pose:
            Pose cartesiana segura general.

        source_pose:
            Pose cartesiana de toma de líquido.

        approach_height:
            Altura de aproximación en metros. Se conserva para compatibilidad,
            pero las transferencias calibradas usan approach_q taught points.
        """

        if approach_height <= 0:
            raise ValueError("approach_height debe ser positivo y estar en metros.")

        self.robot = robot_model
        self.safe_pose = safe_pose
        self.source_pose = source_pose
        self.approach_height = approach_height

    # ---------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------

    def _approach_pose(self, target: SE3) -> SE3:
        """
        Genera una pose elevada sobre el objetivo.

        Como la orientación de la herramienta es constante, se usa un
        desplazamiento positivo en Z respecto a la pose objetivo.
        """

        return target * SE3(0, 0, self.approach_height)

    def _pose_from_q(self, q) -> SE3:
        return self.robot.fkine(np.asarray(q, dtype=float))

    def _z_only_q_for_pose(self, q_approach, target_pose: SE3):
        """
        Construye un punto articular que conserva theta2/theta3 del taught
        approach y solo cambia d1 hasta que el TCP tenga el Z de target_pose.

        Esto evita que IK cambie de rama en movimientos puramente verticales,
        que en hardware deben conservar los servos S2/S3 ya posicionados.
        """

        q_target = np.asarray(q_approach, dtype=float).copy()
        approach_z = float(self._pose_from_q(q_target).t[2])
        target_z = float(target_pose.t[2])
        q_target[0] += target_z - approach_z
        return q_target

    @staticmethod
    def _validate_steps(steps: int) -> None:
        if steps < 2:
            raise ValueError("steps debe ser al menos 2.")

    # ---------------------------------------------------------
    # Segmentos básicos
    # ---------------------------------------------------------

    def move_joint(
        self,
        q_start,
        q_goal,
        steps: int = 50,
        name: str = "joint_motion",
    ) -> MotionSegment:
        """
        Movimiento libre en espacio articular.

        Equivalente conceptual a MoveJ de ABB.
        """

        self._validate_steps(steps)

        traj = free_joint(
            q_start=q_start,
            q_goal=q_goal,
            steps=steps,
        )

        return MotionSegment(
            type="joint",
            trajectory=traj,
            name=name,
        )

    def move_linear(
        self,
        start_pose: SE3,
        target_pose: SE3,
        steps: int = 50,
        name: str = "linear_motion",
    ) -> MotionSegment:
        """
        Movimiento lineal cartesiano.

        Equivalente conceptual a MoveL de ABB.
        """

        self._validate_steps(steps)

        traj = linear_cartesian(
            T_start=start_pose,
            T_goal=target_pose,
            steps=steps,
        )

        return MotionSegment(
            type="cartesian",
            trajectory=traj,
            name=name,
        )

    # ---------------------------------------------------------
    # Compatibilidad con nombres anteriores
    # ---------------------------------------------------------

    def move_home_to_safe(
        self,
        q_home,
        q_safe,
        steps: int = 50,
    ) -> MotionSegment:
        return self.move_joint(
            q_start=q_home,
            q_goal=q_safe,
            steps=steps,
            name="home_to_safe",
        )

    def move_safe_to_target(
        self,
        q_safe,
        q_target_approach,
        steps: int = 5,
    ) -> MotionSegment:
        return self.move_joint(
            q_start=q_safe,
            q_goal=q_target_approach,
            steps=steps,
            name="safe_to_target_approach",
        )

    def linear_approach(
        self,
        start_pose: SE3,
        target_pose: SE3,
        steps: int = 5,
    ) -> MotionSegment:
        return self.move_linear(
            start_pose=start_pose,
            target_pose=target_pose,
            steps=steps,
            name="linear_approach",
        )

    # ---------------------------------------------------------
    # Ciclo de fuente
    # ---------------------------------------------------------

    def plan_source_cycle(
        self,
        q_safe,
        q_source_approach,
        source_pose: Optional[SE3] = None,
        joint_steps: int = 5,
        linear_steps: int = 5,
    ) -> List[MotionSegment]:
        """
        Planifica el ciclo de recolección de líquido:

            Safe
              ↓ MoveJ
            Fuente approach taught point
              ↓ Z-only joint move
            Fuente
              ↑ Z-only joint move
            Fuente approach taught point

        La acción de aspirar debe ejecutarse después del segmento
        "source_down", desde la capa de tarea.
        """

        if source_pose is None:
            source_pose = self.source_pose

        q_source_down = self._z_only_q_for_pose(q_source_approach, source_pose)

        return [
            self.move_joint(
                q_start=q_safe,
                q_goal=q_source_approach,
                steps=joint_steps,
                name="safe_to_source_approach",
            ),

            self.move_joint(
                q_start=q_source_approach,
                q_goal=q_source_down,
                steps=linear_steps,
                name="source_down",
            ),

            self.move_joint(
                q_start=q_source_down,
                q_goal=q_source_approach,
                steps=linear_steps,
                name="source_up",
            ),
        ]

    # ---------------------------------------------------------
    # Ciclo de hoyo
    # ---------------------------------------------------------

    def plan_well_cycle(
        self,
        q_safe,
        q_well_approach,
        well_pose: SE3,
        joint_steps: int = 5,
        linear_steps: int = 5,
    ) -> List[MotionSegment]:
        """
        Planifica el ciclo de deposición en un hoyo:

            Safe
              ↓ MoveJ
            Hoyo approach taught point
              ↓ Z-only joint move
            Hoyo
              ↑ Z-only joint move
            Hoyo approach taught point

        La acción de dispensar debe ejecutarse después del segmento
        "well_down", desde la capa de tarea.
        """

        q_well_down = self._z_only_q_for_pose(q_well_approach, well_pose)

        return [
            self.move_joint(
                q_start=q_safe,
                q_goal=q_well_approach,
                steps=joint_steps,
                name="safe_to_well_approach",
            ),

            self.move_joint(
                q_start=q_well_approach,
                q_goal=q_well_down,
                steps=linear_steps,
                name="well_down",
            ),

            self.move_joint(
                q_start=q_well_down,
                q_goal=q_well_approach,
                steps=linear_steps,
                name="well_up",
            ),
        ]

    # ---------------------------------------------------------
    # Transferencia completa
    # ---------------------------------------------------------

    def plan_transfer(
        self,
        q_safe,
        q_source_approach,
        source_pose: SE3,
        q_target_approach,
        target_pose: SE3,
        joint_steps: int = 5,
        linear_steps: int = 5,
    ) -> List[MotionSegment]:
        """
        Planifica una transferencia completa:

            Safe
              ↓
            Fuente approach
              ↓ Z-only
            Fuente
              ↑ Z-only
            Fuente approach
              ↓ libre
            Safe
              ↓ libre
            Hoyo approach
              ↓ Z-only
            Hoyo
              ↑ Z-only
            Hoyo approach
              ↓ libre
            Safe

        No aspira ni dispensa directamente.
        """

        path: List[MotionSegment] = []

        # Safe -> Fuente -> Fuente approach
        path.extend(
            self.plan_source_cycle(
                q_safe=q_safe,
                q_source_approach=q_source_approach,
                source_pose=source_pose,
                joint_steps=joint_steps,
                linear_steps=linear_steps,
            )
        )

        # Fuente approach -> Safe
        path.append(
            self.move_joint(
                q_start=q_source_approach,
                q_goal=q_safe,
                steps=joint_steps,
                name="source_approach_to_safe",
            )
        )

        # Safe -> Hoyo -> Hoyo approach
        path.extend(
            self.plan_well_cycle(
                q_safe=q_safe,
                q_well_approach=q_target_approach,
                well_pose=target_pose,
                joint_steps=joint_steps,
                linear_steps=linear_steps,
            )
        )

        # Hoyo approach -> Safe
        path.append(
            self.move_joint(
                q_start=q_target_approach,
                q_goal=q_safe,
                steps=joint_steps,
                name="well_approach_to_safe",
            )
        )

        return path
