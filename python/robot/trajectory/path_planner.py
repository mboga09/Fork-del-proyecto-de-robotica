from dataclasses import dataclass
from typing import Any, List, Optional

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
            Modelo del robot. Por ahora se almacena para futura validación,
            pero este planner no llama directamente a fkine/ikine.

        safe_pose:
            Pose cartesiana segura general.

        source_pose:
            Pose cartesiana de toma de líquido.

        approach_height:
            Altura de aproximación en metros.
            0.025 equivale a 25 mm.
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
            Fuente approach
              ↓ MoveL
            Fuente
              ↑ MoveL
            Fuente approach

        La acción de aspirar debe ejecutarse después del segmento
        "source_down", desde la capa de tarea.
        """

        if source_pose is None:
            source_pose = self.source_pose

        source_approach = self._approach_pose(source_pose)

        return [
            self.move_joint(
                q_start=q_safe,
                q_goal=q_source_approach,
                steps=joint_steps,
                name="safe_to_source_approach",
            ),

            self.move_linear(
                start_pose=source_approach,
                target_pose=source_pose,
                steps=linear_steps,
                name="source_down",
            ),

            self.move_linear(
                start_pose=source_pose,
                target_pose=source_approach,
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
            Hoyo approach
              ↓ MoveL
            Hoyo
              ↑ MoveL
            Hoyo approach

        La acción de dispensar debe ejecutarse después del segmento
        "well_down", desde la capa de tarea.
        """

        well_approach = self._approach_pose(well_pose)

        return [
            self.move_joint(
                q_start=q_safe,
                q_goal=q_well_approach,
                steps=joint_steps,
                name="safe_to_well_approach",
            ),

            self.move_linear(
                start_pose=well_approach,
                target_pose=well_pose,
                steps=linear_steps,
                name="well_down",
            ),

            self.move_linear(
                start_pose=well_pose,
                target_pose=well_approach,
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
              ↓ lineal
            Fuente
              ↑ lineal
            Fuente approach
              ↓ libre
            Safe
              ↓ libre
            Hoyo approach
              ↓ lineal
            Hoyo
              ↑ lineal
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