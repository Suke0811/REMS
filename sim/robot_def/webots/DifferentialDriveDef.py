from sim.typing import DefDict, MapRule
from sim.robots import RobotDefBase
from sim.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, Percent
import numpy as np

# space definitions
POS_2D = dict(x=Pos, y=Pos, th_z=float)
VEL_2D = dict(d_x=float, d_y=float, d_th_z=float)
MOTOR_PARAM = dict(pos=Ang(default=float('inf')), vel=AngVel, acc=AngAcc, on=bool, pid=list),
JACOB_2D = dict(jb_x0=float, jb_x1=float,
                jb_y0=float, jb_y1=float,
                jb_w0=float, jb_w1=float)
WHEEEL_VEL = {'wh.l': Percent(scale=(-1, 1)), 'wh.r': Percent(scale=(-1, 1))}

# Key mapping
class KeyMapRule:
    def __init__(self):
        self.arrow = MapRule(
            ['page_up', 'page_down', 'right', 'left', 'up', 'down'], self.arrow_drive, to_list=True)
        self.direct = MapRule(['q', 'e', 'a', 'd'], self.direct_drive, to_list=True)
        self.joy_direct = MapRule(['X', 'Y'], self.direct_drive, to_list=True)

    def get_rules(self):
        return [self.arrow]

    def direct_drive(self, l, r, l_b=None, r_b=None):
        if l_b: l*=-1
        elif r_b: r*=-1
        return 100*l, 100*r

    def joystick_drive(self):
        pass


    def arrow_drive(self, page_up, page_down, right, left, up, down):
        ret = (0, 0)
        if page_up: ret = (-1, 1)
        elif page_down: ret = (1, -1)
        elif up and right: ret = (1, 0.5)
        elif up and left: ret = (0.5, 1)
        elif down and right: ret = (-1, -0.5)
        elif down and left: ret = (-0.5, -1)
        elif right: ret = (1, 0)
        elif left: ret = (0, 1)
        elif up: ret = (1, 1)
        elif down: ret = (-1, -1)
        return self.direct_drive(*ret)


class DifferentialDriveDef(RobotDefBase):
    def __init__(self, radius=0.01, length=0.1, *args, **kwargs):
        """init with a specific initial state (optional) """
        RobotDefBase.__init__(self, *args, **kwargs)
        self.radius = radius
        self.length = length
        self.rule = KeyMapRule()

    def define(self, joint_unit, *args, **kwargs):
        """Definitions of the robot"""
        self.inpt.add_def(WHEEEL_VEL, rules=self.rule.get_rules())
        self.state.add_def(POS_2D)
        self.joint_space.add_def({k: joint_unit for k in self.inpt.keys()}) # same definitino as input but with unit specified
        self.task_space = None
        self.jacobian.add_def(JACOB_2D, shape=(3, 2))
        super().define()


    def jb(self, theta: DefDict, *args, **kwargs):
        th = theta.get('theta')
        r = self.radius
        l = self.length
        jacobian = np.array([r*np.cos(th), r*np.cos(th),
                             r*np.sin(th), r*np.sin(th),
                             -1/l, 1/l])
        return self.jacobian.format(jacobian)


