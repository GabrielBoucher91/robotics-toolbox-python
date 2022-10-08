import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, RevoluteMDH


class Meca500(DHRobot):
    """
    Class that models a Mecademic Meca500 manipulator

    ``Meca500()`` is an object which models a Meca500 robot and
    describes its kinematic and dynamic characteristics using standard DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.DH.Meca500()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration

    .. note::
        - SI units are used.

    :References:

        - `Parameters for calculations of kinematics can be given on request at support@mecademic.com`_



    .. codeauthor:: Gabriel Boucher
    """  # noqa

    def __init__(self):

        from math import pi
        zero = 0.0

        deg = pi / 180
        inch = 0.0254

        # Robot links setup
        L0 = RevoluteMDH(
            d=135,
            a=0,
            alpha=0,
            offset=0
        )

        L1 = RevoluteMDH(
            d=0,
            a=0,
            alpha=-90*deg,
            offset=-90*deg
        )

        L2 = RevoluteMDH(
            d=0,
            a=135,
            alpha=0,
            offset=0
        )

        L3 = RevoluteMDH(
            d=120,
            a=38,
            alpha=-90*deg,
            offset=0
        )

        L4 = RevoluteMDH(
            d=0,
            a=0,
            alpha=90*deg,
            offset=0
        )

        L5 = RevoluteMDH(
            d=70,
            a=0,
            alpha=-90*deg,
            offset=180*deg
        )

        super().__init__(
            [L0, L1, L2, L3, L4, L5],
            name="Meca500",
            manufacturer="Mecademic Industrial Robotics",
        )

        self.qz = np.zeros(6)

        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover
    import roboticstoolbox as rbt
    import math
    meca500 = Meca500()
    Ts = meca500.fkine(meca500.qz)
    print(Ts)
    Te = meca500.fkine([0,0,0,0,math.pi/2, 0])
    print(Te)
    traj = rbt.jtraj(meca500.qz, [math.pi/2,0,0,0,math.pi/2,0], 50)
    meca500.plot(traj.q, backend='pyplot')
