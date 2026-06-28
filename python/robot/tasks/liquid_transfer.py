import numpy as np

from robot.debug.plot_diagnostics import generate_transfer_debug_plots


class LiquidTransferTask:
    def __init__(self, planner, executor, motion_sender, layout, status_callback=None, generate_debug_plots: bool = True):
        self.planner = planner
        self.executor = executor
        self.motion_sender = motion_sender
        self.layout = layout
        self.status_callback = status_callback
        self.generate_debug_plots = generate_debug_plots

    def run_wells(self, wells: list[str]) -> None:
        if not wells:
            raise ValueError("No wells selected.")

        self._emit(f"Starting transfer to wells: {wells}")

        if self.generate_debug_plots:
            self._emit("Generating model and trajectory diagnostic plots.")
            generate_transfer_debug_plots(
                robot=self.executor.robot,
                planner=self.planner,
                layout=self.layout,
                q_initial=self.executor.current_q.copy(),
                wells=wells,
                status_callback=self._emit,
            )

        self._ensure_safe_position()

        for index, well_id in enumerate(wells):
            self._emit(f"Transfer to {well_id}")
            self._run_single_transfer(well_id, transfer_index=index)

        self._emit("Transfer finished.")

    def _run_single_transfer(self, well_id: str, transfer_index: int) -> None:
        q_safe = self.layout.q_safe()
        source_pose = self.layout.source_pose_for_transfer(transfer_index)
        target_pose = self.layout.well_pose(well_id)

        # Los puntos de approach son taught points del layout. Esto garantiza que
        # HOME->INTRODUCIR_JERINGA y HOME->RACK usen las duraciones Z medidas,
        # sin que IK/approach_height cambien la altura calibrada.
        q_source_approach = self.layout.source_approach_q()
        q_target_approach = self.layout.well_approach_q(well_id)

        self._emit(f"Source Z for pass {transfer_index + 1}: {float(source_pose.t[2]):.4f} m")

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
            self._emit(f"Segment: {segment.name}")
            self.executor.execute_segment(segment)

            if segment.name == "source_down":
                self._emit("Tool aspirate.")
                self.motion_sender.aspirate()

            elif segment.name == "well_down":
                self._emit(f"Tool dispense at {well_id}.")
                self.motion_sender.dispense()

    def _ensure_safe_position(self) -> None:
        q_safe = self.layout.q_safe()

        if np.allclose(self.executor.current_q, q_safe, atol=1e-6):
            return

        self._emit("Moving to safe position.")

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
