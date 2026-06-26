import math
from dataclasses import dataclass

import numpy as np

L1_M = 0.150
L2_M = 0.150
Q2_MIN_DEG = -30.0
Q2_MAX_DEG = 30.0
Q3_MIN_DEG = 0.0
Q3_MAX_DEG = 90.0
WELL_SPACING_M = 0.039

WELL_LOCAL_XY = {
    "A1": (-WELL_SPACING_M, 0.0),
    "A2": (0.0, 0.0),
    "A3": (WELL_SPACING_M, 0.0),
    "B1": (-WELL_SPACING_M, -WELL_SPACING_M),
    "B2": (0.0, -WELL_SPACING_M),
    "B3": (WELL_SPACING_M, -WELL_SPACING_M),
}


@dataclass
class JointSolution:
    q2_deg: float
    q3_deg: float
    margin_deg: float


@dataclass
class RackCandidate:
    a2_x_m: float
    a2_y_m: float
    theta_deg: float
    min_margin_deg: float


def reachable_radius_limits() -> tuple[float, float]:
    q3_max = math.radians(max(abs(Q3_MIN_DEG), abs(Q3_MAX_DEG)))
    r_min = math.sqrt(L1_M**2 + L2_M**2 + 2.0 * L1_M * L2_M * math.cos(q3_max))
    r_max = L1_M + L2_M
    return r_min, r_max


def ik_solutions(x_m: float, y_m: float) -> list[JointSolution]:
    r2 = x_m * x_m + y_m * y_m
    cos_q3 = (r2 - L1_M**2 - L2_M**2) / (2.0 * L1_M * L2_M)

    if cos_q3 < -1.0 or cos_q3 > 1.0:
        return []

    cos_q3 = max(-1.0, min(1.0, cos_q3))
    raw_q3 = math.acos(cos_q3)
    candidates = [raw_q3, -raw_q3]

    valid = []
    for q3 in candidates:
        q2 = math.atan2(y_m, x_m) - math.atan2(
            L2_M * math.sin(q3),
            L1_M + L2_M * math.cos(q3),
        )
        q2 = (q2 + math.pi) % (2.0 * math.pi) - math.pi

        q2_deg = math.degrees(q2)
        q3_deg = math.degrees(q3)

        margin = min(
            q2_deg - Q2_MIN_DEG,
            Q2_MAX_DEG - q2_deg,
            q3_deg - Q3_MIN_DEG,
            Q3_MAX_DEG - q3_deg,
        )

        if margin >= 0.0:
            valid.append(JointSolution(q2_deg=q2_deg, q3_deg=q3_deg, margin_deg=margin))

    valid.sort(key=lambda sol: sol.margin_deg, reverse=True)
    return valid


def rack_points(a2_x_m: float, a2_y_m: float, theta_deg: float) -> dict[str, tuple[float, float]]:
    theta = math.radians(theta_deg)
    c = math.cos(theta)
    s = math.sin(theta)

    points = {}
    for name, (local_x, local_y) in WELL_LOCAL_XY.items():
        x = a2_x_m + c * local_x - s * local_y
        y = a2_y_m + s * local_x + c * local_y
        points[name] = (x, y)

    return points


def validate_rack(a2_x_m: float, a2_y_m: float, theta_deg: float) -> tuple[bool, float, dict[str, JointSolution | None]]:
    solutions = {}
    margins = []

    for well_id, (x, y) in rack_points(a2_x_m, a2_y_m, theta_deg).items():
        point_solutions = ik_solutions(x, y)
        if not point_solutions:
            solutions[well_id] = None
            margins.append(-999.0)
            continue

        solutions[well_id] = point_solutions[0]
        margins.append(point_solutions[0].margin_deg)

    min_margin = min(margins)
    return min_margin >= 0.0, min_margin, solutions


def search_rack_pose(
    x_min: float = 0.18,
    x_max: float = 0.32,
    y_min: float = -0.16,
    y_max: float = 0.16,
    theta_min: float = 0.0,
    theta_max: float = 180.0,
    xy_step: float = 0.001,
    theta_step: float = 1.0,
) -> RackCandidate | None:
    best = None

    xs = np.arange(x_min, x_max + 0.5 * xy_step, xy_step)
    ys = np.arange(y_min, y_max + 0.5 * xy_step, xy_step)
    thetas = np.arange(theta_min, theta_max + 0.5 * theta_step, theta_step)

    for theta in thetas:
        for x in xs:
            for y in ys:
                ok, min_margin, _ = validate_rack(float(x), float(y), float(theta))
                if not ok:
                    continue

                candidate = RackCandidate(
                    a2_x_m=float(x),
                    a2_y_m=float(y),
                    theta_deg=float(theta),
                    min_margin_deg=float(min_margin),
                )

                if best is None or candidate.min_margin_deg > best.min_margin_deg:
                    best = candidate

    return best


def print_candidate(candidate: RackCandidate) -> None:
    r_min, r_max = reachable_radius_limits()
    print(f"Reachable radius band: {r_min:.6f} m to {r_max:.6f} m")
    print(candidate)

    ok, min_margin, solutions = validate_rack(candidate.a2_x_m, candidate.a2_y_m, candidate.theta_deg)
    print(f"valid={ok}, min_margin_deg={min_margin:.3f}")

    points = rack_points(candidate.a2_x_m, candidate.a2_y_m, candidate.theta_deg)
    for well_id, (x, y) in points.items():
        solution = solutions[well_id]
        r = math.hypot(x, y)
        if solution is None:
            print(f"{well_id}: x={x:.6f}, y={y:.6f}, r={r:.6f}, invalid")
        else:
            print(
                f"{well_id}: x={x:.6f}, y={y:.6f}, r={r:.6f}, "
                f"q2={solution.q2_deg:.3f} deg, "
                f"q3={solution.q3_deg:.3f} deg, "
                f"margin={solution.margin_deg:.3f} deg"
            )


if __name__ == "__main__":
    candidate = search_rack_pose()
    if candidate is None:
        print("No valid rack pose found.")
    else:
        print_candidate(candidate)
