import numpy as np
from spatialmath import SE3

from tools.config_loader import load_yaml_config


class LabLayout:
    def __init__(self, config_file: str = "workspace_config.yaml"):
        self.config = load_yaml_config(config_file)

    def q_home(self):
        return self._q_from_dict(self.config["q_home"])

    def q_safe(self):
        return self._q_from_dict(self.config["q_safe"])

    def source_pose(self) -> SE3:
        return self._pose_from_dict(self.config["source"]["pose"])

    def source_pose_for_transfer(self, transfer_index: int) -> SE3:
        if transfer_index < 0:
            raise ValueError("transfer_index must be non-negative.")

        data = dict(self.config["source"]["pose"])
        data["z"] = float(data["z"]) - transfer_index * self.source_depth_step_m()
        return self._pose_from_dict(data)

    def source_depth_step_m(self) -> float:
        return float(self.config.get("source", {}).get("depth_step_m", 0.0))

    def source_approach_q(self):
        return self._q_from_dict(self.config["source"]["approach_q"])

    def well_pose(self, well_id: str) -> SE3:
        return self._pose_from_dict(self._well_config(well_id)["pose"])

    def well_approach_q(self, well_id: str):
        return self._q_from_dict(self._well_config(well_id)["approach_q"])

    def all_wells(self) -> list[str]:
        return list(self.config["wells"].keys())

    def joint_steps(self) -> int:
        return int(self.config.get("robot", {}).get("joint_steps", 5))

    def linear_steps(self) -> int:
        return int(self.config.get("robot", {}).get("linear_steps", 2))

    def approach_height_m(self) -> float:
        return float(self.config.get("robot", {}).get("approach_height_m", 0.025))

    def _well_config(self, well_id: str) -> dict:
        wells = self.config.get("wells", {})

        if well_id not in wells:
            raise ValueError(f"Well not configured in workspace_config.yaml: {well_id}")

        return wells[well_id]

    @staticmethod
    def _q_from_dict(data: dict):
        return np.array([
            float(data["d1"]),
            np.deg2rad(float(data["theta2_deg"])),
            np.deg2rad(float(data["theta3_deg"])),
        ])

    @staticmethod
    def _pose_from_dict(data: dict) -> SE3:
        return SE3(
            float(data["x"]),
            float(data["y"]),
            float(data["z"]),
        )
