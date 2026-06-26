import math
from dataclasses import dataclass

import numpy as np


L2_M = 0.150
L3_M = 0.150
Q3_LIMIT_DEG = 30.0
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
class RackCandidate:
    a2_x_m: float
    a2_y_m: float
    theta_deg: float
    max_violation_m: float
    total_violation_m: float
    min_radius_m: float
    max_radius_m: float


def reachable_radius_limits() -> tuple[float, float]:
    q3 = math.radians(Q3_LIMIT_DEG)
    r_min = math.sqrt(L2_M**2 + L3_M**2 + 2.0 * L2_M * L3_M * math.cos(q3))
    r_max = L2_M + L3_M
    return r_min, r_max


def q3_required_abs_deg(x_m: float, y_m: float) -> float | None:
    r = math.hypot(x_m, y_m)
    cos_q3 = (r**2 - L2_M**2 - L3_M**2) / (2.0 * L2_M * L3_M)
    if cos_q3 < -1.0 or cos_q3 > 1.0:
        return None
    return math.degrees(math.acos(cos_q3))


def point_is_reachable(x_m: float, y_m: float) -> bool:
    q3 = q3_required_abs_deg(x_m, y_m)
    return q3 is not None and q3 <= Q3_LIMIT_DEG


def rack_points(a2_x_m: float, a2_y_m: float, theta_deg: float) -> dict[str, tuple[float, float]]:
    theta = math.radians(theta_deg)
    c = math.cos(theta)
    s = math.sin(theta)

    points = {}
    for name, (lx, ly) in WELL_LOCAL_XY.items():
        x = a2_x_m + c * lx - s * ly
        y = a2_y_m + s * lx + c * ly
        points[name] = (x, y)
    return points


def rack_violation(a2_x_m: float, a2_y_m: float, theta_deg: float) -> tuple[float, float, float, float]:
    r_min, r_max = reachable_radius_limits()
    radii = []
    violations = []

    for x, y in rack_points(a2_x_m, a2_y_m, theta_deg).values():
        r = math.hypot(x, y)
        radii.append(r)
        violations.append(max(r_min - r, 0.0) + max(r - r_max, 0.0))

    return max(violations), sum(violations), min(radii), max(radii)


def search_rack_pose(
    x_min: float = 0.20,
    x_max: float = 0.36,
    y_min: float = -0.16,
    y_max: float = 0.16,
    theta_min: float = 0.0,
    theta_max: float = 180.0,
    xy_step: float = 0.001,
    theta_step: float = 1.0,
) -> RackCandidate:
    best = None

    xs = np.arange(x_min, x_max + 0.5 * xy_step, xy_step)
    ys = np.arange(y_min, y_max + 0.5 * xy_step, xy_step)
    thetas = np.arange(theta_min, theta_max + 0.5 * theta_step, theta_step)

    for theta in thetas:
        for x in xs:
            for y in ys:
                max_v, total_v, min_r, max_r = rack_violation(float(x), float(y), float(theta))
                candidate = RackCandidate(float(x), float(y), float(theta), max_v, total_v, min_r, max_r)
                if best is None:
                    best = candidate
                    continue
                if (candidate.max_violation_m, candidate.total_violation_m) < (best.max_violation_m, best.total_violation_m):
                    best = candidate

    return best


def print_candidate(candidate: RackCandidate) -> None:
    r_min, r_max = reachable_radius_limits()
    print(f"Reachable radius band: {r_min:.6f} m to {r_max:.6f} m")
    print(candidate)

    points = rack_points(candidate.a2_x_m, candidate.a2_y_m, candidate.theta_deg)
    for name, (x, y) in points.items():
        r = math.hypot(x, y)
        q3 = q3_required_abs_deg(x, y)
        ok = point_is_reachable(x, y)
        q3_text = "None" if q3 is None else f"{q3:.3f} deg"
        print(f"{name}: x={x:.6f}, y={y:.6f}, r={r:.6f}, q3_abs={q3_text}, ok={ok}")


if __name__ == "__main__":
    candidate = search_rack_pose()
    print_candidate(candidate)
