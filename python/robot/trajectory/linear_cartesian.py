import roboticstoolbox as rtb
from spatialmath import SE3


def linear_cartesian(T_start: SE3, T_goal: SE3, steps: int = 50):
    """
    Genera una trayectoria cartesiana lineal usando ctraj.

    Equivalente conceptual a MoveL de ABB.

    Parameters
    ----------
    T_start:
        Pose cartesiana inicial como SE3.

    T_goal:
        Pose cartesiana final como SE3.

    steps:
        Número de puntos discretos de la trayectoria.

    Returns
    -------
    list[SE3]:
        Lista de poses SE3 interpoladas.
    """

    if steps < 2:
        raise ValueError("steps debe ser al menos 2.")

    return rtb.ctraj(T_start, T_goal, steps)