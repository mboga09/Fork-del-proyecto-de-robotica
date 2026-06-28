from roboticstoolbox import DHRobot
from roboticstoolbox import RevoluteMDH
from roboticstoolbox import PrismaticMDH
from spatialmath import SE3


class ScaraPRR(DHRobot):
    def __init__(self):
        L2 = 0.150
        L3 = 0.150

        links = [
            # Robotics Toolbox requires a finite qlim for prismatic joints.
            # This is only a numerical placeholder; Z is not validated in Python.
            PrismaticMDH(theta=0, a=0, alpha=0, qlim=[-10.0, 10.0]),
            # Angular joints intentionally have no software limits in the model.
            RevoluteMDH(d=0, a=0, alpha=0),
            RevoluteMDH(d=0, a=L2, alpha=0),
        ]

        super().__init__(links, name="SCARA_PRR")

        self.tool = SE3(L3, 0, 0) * SE3(0, 0, -0.024)
        self.base = SE3(0, 0, 0.0025)
