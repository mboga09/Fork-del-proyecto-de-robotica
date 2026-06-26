import numpy as np


class LiquidTransferTask:
    """
    Ejecuta la tarea completa de transferencia.

    Esta clase coordina:
        - PathPlanner
        - MotionExecutor
        - JsonMotionSender para aspirar / dispensar

    No conoce PySide6.
    No conoce botones.
    """

    def __init__(
        self,
        planner,
        executor,
        motion_sender,
        layout,
        status_callback=None,
    ):
        self.planner = planner
        self.executor = executor
        self.motion_sender = motion_sender
        self.layout = layout
        self.status_callback = status_callback

    def run_wells(self, wells: list[str]) -> None:
        if not wells:
            raise ValueError("No hay wells seleccionados.")

        self._emit(f"Iniciando transferencia a wells: {wells}")

        self._ensure_safe_position()

        for well_id in wells:
            self._emit(f"Transferencia hacia {well_id}")
            self._run_single_transfer(well_id)

        self._emit("Transferencia finalizada.")

    def _run_single_transfer(self, well_id: str) -> None:
        q_safe = self.layout.q_safe()
        q_source_approach = self.layout.source_approach_q()
        q_target_approach = self.layout.well_approach_q(well_id)

        source_pose = self.layout.source_pose()
        target_pose = self.layout.well_pose(well_id)

        path = self.planner.plan_transfer(
            q_safe=q_safe,
            q_source_approach=q_source_approach,
            source_pose=source_pose,
            q_target_approach=q_target_approach,
            target_pose=target_pose,
            joint_steps=self.layout.joint_steps(),
            linear_steps=self.layout.linear_steps(),
        )

        for segment in path:
            self._emit(f"Segmento: {segment.name}")
            self.executor.execute_segment(segment)

            if segment.name == "source_down":
                self._emit("Aspirando líquido.")
                self.motion_sender.aspirate()

            elif segment.name == "well_down":
                self._emit(f"Dispensando líquido en {well_id}.")
                self.motion_sender.dispense()

    def _ensure_safe_position(self) -> None:
        """
        Si el executor no cree estar en q_safe, genera un MoveJ desde
        la configuración actual hasta q_safe.
        """

        q_safe = self.layout.q_safe()

        if np.allclose(self.executor.current_q, q_safe, atol=1e-6):
            return

        self._emit("Moviendo a posición segura.")

        segment = self.planner.move_joint(
            q_start=self.executor.current_q,
            q_goal=q_safe,
            steps=self.layout.joint_steps(),
            name="current_to_safe",
        )

        self.executor.execute_segment(segment)

    def _emit(self, message: str) -> None:
        if self.status_callback is not None:
            self.status_callback(message)