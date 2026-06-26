def build_home_command() -> str:
    return "HOME\n"


def build_stop_command() -> str:
    return "STOP\n"


def build_move_joints_command(q1: float, q2: float, q3: float) -> str:
    return f"MOVE_JOINTS,{q1:.5f},{q2:.5f},{q3:.5f}\n"