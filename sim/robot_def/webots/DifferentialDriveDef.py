from sim.typing import DefDict, BindRule
from sim.robots import RobotDefBase
from sim.inputs.map.JOYSTICK_KEYMAP import StdKeys
import numpy as np

# space definitions
POS_2D = dict(x=float, y=float, th_y=float)
VEL_2D = dict(d_x=float, d_y=float, d_th_y=float)
MOTOR_PARAM = dict(pos=float('inf'), vel=float, acc=float, on=bool, pid=list),
JACOB_2D = dict(jb_x0=float, jb_x1=float,
                jb_y0=float, jb_y1=float,
                jb_w0=float, jb_w1=float)
WHEEEL_VEL = dict(wh_l=float, wh_r=float)

# Key mapping
class KeyMapRule:
    def __init__(self, max_vel):
        self.max_vel = max_vel
        self.arrow = BindRule(
            ['page_up', 'page_down', 'right', 'left', 'up', 'down'], self.arrow_drive, target='keyboard')
        self.direct = BindRule(['q', 'e', 'a', 'd'], self.direct_drive, target='keyboard')
        self.joy_direct = BindRule(['X', 'Y'], self.direct_drive, target='joystick')

    def get_rules(self):
        return [self.arrow]

    def direct_drive(self, l, r, l_b=None, r_b=None):
        if l_b: l*-1
        elif r_b: r*-1
        return l*self.max_vel, r*self.max_vel

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
    def __init__(self, radius=0.01, length=0.1, max_vel=1.0, *args, **kwargs):
        """init with a specific initial state (optional) """
        RobotDefBase.__init__(self, *args, **kwargs)
        self.radius = radius
        self.length = length
        self.rule = KeyMapRule(max_vel)

    def define(self, drive_names, sensor_names, *args, **kwargs):
        """Definitions of the robot"""
        self.inpt = DefDict(WHEEEL_VEL, rules=self.rule.get_rules())
        self.state = DefDict((POS_2D,))
        self.outpt = DefDict(sensor_names)
        self.joint_space = DefDict(drive_names, suffixes=['pos', 'vel', 'acc', 'on', 'pid'], rules=BindRule(WHEEEL_VEL, None,))
        self.task_space = None
        self.jacobian = DefDict(JACOB_2D, shape=(3, 2))
        super().define()

    def control(self, inpt, timestamp):
        self.inpt.set(inpt)
        self.joint_space.vel().set(inpt)
        return self.joint_space

    def jb(self, theta: DefDict, *args, **kwargs):
        th = theta.get('theta')
        r = self.radius
        l = self.length
        jacobian = np.array([r*np.cos(th), r*np.cos(th),
                             r*np.sin(th), r*np.sin(th),
                             -1/l, 1/l])
        return self.jacobian.format(jacobian)


