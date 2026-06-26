from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Iterable

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from spatialmath import SE3

from tools.config_loader import PROJECT_ROOT


DEBUG_PLOTS_DIR = PROJECT_ROOT / "debug_plots"


def generate_transfer_debug_plots(
    *,
    robot,
    planner,
    layout,
    q_initial,
    wells: list[str],
    status_callback=None,
) -> tuple[Path, Path]:
    """
    Genera dos PNGs para diagnosticar el pipeline antes de mover el robot:

    1. Modelo 3D del robot en q_initial y en la pose final del último well.
    2. Puntos 3D de la trayectoria generada por PathPlanner, unidos con flechas.
    """
    if not wells:
        raise ValueError("No hay wells para graficar.")

    DEBUG_PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    q_initial = np.asarray(q_initial, dtype=float)
    last_well = wells[-1]
    final_pose = layout.well_pose(last_well)
    q_final = _ik_or_fallback(robot, final_pose, layout.well_approach_q(last_well))

    robot_plot_path = DEBUG_PLOTS_DIR / f"robot_initial_final_{timestamp}.png"
    path_plot_path = DEBUG_PLOTS_DIR / f"trajectory_arrows_3d_{timestamp}.png"

    _plot_robot_initial_final(robot, q_initial, q_final, last_well, robot_plot_path)

    points = _collect_path_points(
        robot=robot,
        planner=planner,
        layout=layout,
        q_start=q_initial,
        wells=wells,
    )
    _plot_path_arrows_3d(points, path_plot_path)

    _emit(status_callback, f"Debug plot robot: {robot_plot_path}")
    _emit(status_callback, f"Debug plot trayectoria: {path_plot_path}")

    return robot_plot_path, path_plot_path


def _collect_path_points(*, robot, planner, layout, q_start, wells: list[str]) -> np.ndarray:
    points: list[np.ndarray] = []
    q_cursor = np.asarray(q_start, dtype=float)

    # Punto inicial real estimado desde fkine.
    points.append(_tcp_from_q(robot, q_cursor))

    q_safe = layout.q_safe()

    if not np.allclose(q_cursor, q_safe, atol=1e-6):
        segment = planner.move_joint(
            q_start=q_cursor,
            q_goal=q_safe,
            steps=layout.joint_steps(),
            name="debug_current_to_safe",
        )
        _append_segment_points(points, robot, segment)
        q_cursor = q_safe

    for well_id in wells:
        path = planner.plan_transfer(
            q_safe=q_safe,
            q_source_approach=layout.source_approach_q(),
            source_pose=layout.source_pose(),
            q_target_approach=layout.well_approach_q(well_id),
            target_pose=layout.well_pose(well_id),
            joint_steps=layout.joint_steps(),
            linear_steps=layout.linear_steps(),
        )

        for segment in path:
            _append_segment_points(points, robot, segment)

        q_cursor = q_safe

    return np.asarray(points, dtype=float)


def _append_segment_points(points: list[np.ndarray], robot, segment) -> None:
    if segment.type == "joint":
        for q in segment.trajectory.q:
            points.append(_tcp_from_q(robot, q))
        return

    if segment.type == "cartesian":
        for pose in segment.trajectory:
            points.append(np.asarray(pose.t, dtype=float))
        return

    raise ValueError(f"Tipo de segmento desconocido: {segment.type}")


def _plot_robot_initial_final(robot, q_initial, q_final, last_well: str, output_path: Path) -> None:
    initial_points = _robot_link_points(robot, q_initial)
    final_points = _robot_link_points(robot, q_final)

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        initial_points[:, 0],
        initial_points[:, 1],
        initial_points[:, 2],
        marker="o",
        label="q inicial",
    )
    ax.plot(
        final_points[:, 0],
        final_points[:, 1],
        final_points[:, 2],
        marker="o",
        label=f"pose final {last_well}",
    )

    ax.scatter(initial_points[-1, 0], initial_points[-1, 1], initial_points[-1, 2], label="TCP inicial")
    ax.scatter(final_points[-1, 0], final_points[-1, 1], final_points[-1, 2], label="TCP final")

    ax.set_title("Modelo PRR: configuración inicial vs final")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend()
    _set_equal_axes_3d(ax, np.vstack([initial_points, final_points]))

    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def _plot_path_arrows_3d(points: np.ndarray, output_path: Path) -> None:
    if points.shape[0] < 2:
        raise ValueError("Se necesitan al menos dos puntos para graficar flechas.")

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(points[:, 0], points[:, 1], points[:, 2], marker="o", markersize=3)

    deltas = points[1:] - points[:-1]
    ax.quiver(
        points[:-1, 0],
        points[:-1, 1],
        points[:-1, 2],
        deltas[:, 0],
        deltas[:, 1],
        deltas[:, 2],
        length=1.0,
        normalize=False,
        arrow_length_ratio=0.2,
    )

    ax.scatter(points[0, 0], points[0, 1], points[0, 2], s=60, label="inicio")
    ax.scatter(points[-1, 0], points[-1, 1], points[-1, 2], s=60, label="final")

    ax.set_title("Trayectoria 3D generada por PathPlanner")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend()
    _set_equal_axes_3d(ax, points)

    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def _ik_or_fallback(robot, pose: SE3, q_seed) -> np.ndarray:
    try:
        sol = robot.ikine_LM(
            pose,
            q0=q_seed,
            mask=[1, 1, 1, 0, 0, 0],
            joint_limits=True,
        )
        if sol.success:
            return np.asarray(sol.q, dtype=float)
    except Exception:
        pass

    return np.asarray(q_seed, dtype=float)


def _tcp_from_q(robot, q) -> np.ndarray:
    return np.asarray(robot.fkine(np.asarray(q, dtype=float)).t, dtype=float)


def _robot_link_points(robot, q) -> np.ndarray:
    q = np.asarray(q, dtype=float)

    d1 = float(q[0])
    theta2 = float(q[1])
    theta3 = float(q[2])

    base_z = float(robot.base.t[2]) if hasattr(robot, "base") else 0.0
    l2 = float(getattr(robot.links[2], "a", 0.150))
    l3 = float(robot.tool.t[0]) if hasattr(robot, "tool") else 0.150
    tool_z = float(robot.tool.t[2]) if hasattr(robot, "tool") else -0.024

    p0 = np.array([0.0, 0.0, base_z])
    p1 = np.array([0.0, 0.0, base_z + d1])
    p2 = p1 + np.array([
        l2 * np.cos(theta2),
        l2 * np.sin(theta2),
        0.0,
    ])
    p3 = p2 + np.array([
        l3 * np.cos(theta2 + theta3),
        l3 * np.sin(theta2 + theta3),
        tool_z,
    ])

    return np.vstack([p0, p1, p2, p3])


def _set_equal_axes_3d(ax, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    centers = (mins + maxs) / 2.0
    radius = max(float((maxs - mins).max()) / 2.0, 0.05)

    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)


def _emit(status_callback, message: str) -> None:
    if status_callback is not None:
        status_callback(message)
    else:
        print(message, flush=True)
