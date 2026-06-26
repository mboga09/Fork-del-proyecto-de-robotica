import numpy as np
import roboticstoolbox as rtb


def free_joint(q_start, q_goal, steps: int = 50):
    """
    Genera una trayectoria libre en espacio articular usando jtraj.

    Equivalente conceptual a MoveJ de ABB.

    Parameters
    ----------
    q_start:
        Configuración articular inicial [d1, theta2, theta3].

    q_goal:
        Configuración articular final [d1, theta2, theta3].

    steps:
        Número de puntos discretos de la trayectoria.

    Returns
    -------
    Trajectory:
        Objeto devuelto por roboticstoolbox.jtraj.
        Contiene q, qd y qdd.
    """

    if steps < 2:
        raise ValueError("steps debe ser al menos 2.")

    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)

    if q_start.shape != q_goal.shape:
        raise ValueError("q_start y q_goal deben tener la misma dimensión.")

    return rtb.jtraj(q_start, q_goal, steps)