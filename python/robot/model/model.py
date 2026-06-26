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

            PrismaticMDH(
                theta=0,
                a=0,
                alpha=0,
                qlim=[0.0, 0.4]
            ),

            RevoluteMDH(
                d=0,
                a=0,
                alpha=0,
                # Diagnóstico de workspace:
                # El layout actual contiene puntos a ambos lados del eje X y
                # requiere q2 negativos para el reservorio espejado.
                qlim=[-np.pi, np.pi]
            ),

            RevoluteMDH(
                d=0,
                a=L2,
                alpha=0,
                # El límite anterior [-30°, 30°] no alcanza los puntos del
                # plato ni del reservorio con L2=L3=150 mm. El rango 0°..150°
                # permite validar la cinemática y el pathing en esta rama de
                # diagnóstico. La calibración física del actuador J3 se debe
                # cerrar antes de pruebas con hardware conectado.
                qlim=[0.0, np.deg2rad(150.0)]
            )
        ]

        super().__init__(
            links,
            name="SCARA_PRR"
        )

        self.tool = (
            SE3(L3, 0, 0)
            * SE3(0, 0, -0.024)
        )

        self.base = SE3(0, 0, 0.0025)
