import numpy as np

from roboticstoolbox import DHRobot
from roboticstoolbox import RevoluteMDH
from roboticstoolbox import PrismaticMDH
from spatialmath import SE3

class ScaraPRR(DHRobot):
    def __init__(self):
        L2 = 0.150
        L3 = 0.150

        links = [
            PrismaticMDH(theta=0, a=0, alpha=0, qlim=[0.0, 0.4]),
            RevoluteMDH(d=0, a=0, alpha=0, qlim=[-np.pi / 2, np.pi * 3 / 2]),
            RevoluteMDH(d=0, a=L2, alpha=0, qlim=[-np.pi / 6, np.pi / 6]),
        ]

        super().__init__(links, name="SCARA_PRR")

        self.tool = SE3(L3, 0, 0) * SE3(0, 0, -0.024)
        self.base = SE3(0, 0, 0.0025)
