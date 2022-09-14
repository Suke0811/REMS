from sim.typing import DefDict, MapRule
from sim.robots import RobotDefBase
from sim.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, Percent
import numpy as np

# space definitions
POS_2D = dict(x=Pos, y=Pos, th_z=float)
VEL_2D = dict(d_x=float, d_y=float, d_th_z=float)
MOTOR_PARAM = dict(pos=Ang(default=float('inf')), vel=AngVel, acc=AngAcc, on=bool, pid=list),
JACOB_2D = dict(jb_x0=float, jb_x1=float, jb_x2=float, jb_x3=float,
                jb_y0=float, jb_y1=float, jb_y2=float, jb_y3=float,
                jb_w0=float, jb_w1=float, jb_w2=float, jb_w3=float)

WHEEEL_VEL = {'wh.0': Percent(scale=(-1, 1)), 'wh.1': Percent(scale=(-1, 1)),
              'wh.2': Percent(scale=(-1, 1)), 'wh.3': Percent(scale=(-1, 1))}

ARROW_VEL = {'th_z': Percent(scale=(-1, 1)), 'd_x': Percent(scale=(-1, 1)), 'd_y': Percent(scale=(-1, 1))}



# Key mapping
class KeyMapRule:
    def __init__(self, arrow_units):
        self.jacobian = None
        self.arrow_percent = DefDict(ARROW_VEL)
        self.arrow = MapRule(
            ['page_up', 'page_down', 'right', 'left', 'up', 'down'], self.arrow_drive, ARROW_VEL, to_list=True)
        self.direct = MapRule(['q', 'e', 'a', 'd'], self.direct_drive, to_list=True)
        self.joy_direct = MapRule(['X', 'Y'],
                                  self.direct_drive,
                                  {'wh.l': Percent(scale=(-1, 1)), 'wh.r': Percent(scale=(-1, 1))},
                                  to_list=True)

    def get_rules(self):
        return [self.arrow]

    def direct_drive(self, vel_2d):
        jb = self.jacobian().ndarray()
        u = np.transpose(jb) @ vel_2d.ndarray().reshape((3, 1))
        return u.flatten()

    def joystick_drive(self):
        pass

    def arrow_drive(self, page_up, page_down, right, left, up, down):
        ret = np.array([0, 0, 0])
        if page_up: ret += np.array([1, 0, 0])
        if page_down: ret += np.array([-1, 0, 0])
        if up: ret += np.array([0, 1, 0])
        if down: ret += np.array([0, -1, 0])
        if right: ret += np.array([0, 0, 1])
        if left: ret += np.array([0, 0, -1])
        self.arrow_percent.set(100*ret)
        return self.arrow_percent


class MecanumDriveDef(RobotDefBase):
    def __init__(self, radius=0.01, length=0.1, width=0.1, *args, **kwargs):
        """init with a specific initial state (optional) """
        RobotDefBase.__init__(self, *args, **kwargs)
        self.radius = radius
        self.length = length
        self.width = width

    def define(self, arrow_units, inpt_unit=AngVel(drange=(-10, 10)), drive_space=None, sense_space=None, *args, **kwargs):
        """Definitions of the robot"""

        self.rule = KeyMapRule(arrow_units)
        self.inpt.add_def({k: u for k, u in zip(ARROW_VEL.keys(), arrow_units.values())},  rules=self.rule.arrow) # same definitino as input but with unit specified
        self.state.add_def(POS_2D)
        if drive_space is not None:
            self.drive_space.add_def(drive_space)
        if sense_space is not None:
            self.sense_space.add_def(sense_space)
        self.jacobian.add_def(JACOB_2D, shape=(3, 4))
        self.joint_space = None
        self.task_space = None
        super().define()

    # def drive(self, inpt, t, *args, **kwargs):
    #     inpt =
    #     super().drive(inpt, t, args, kwargs)

    def jb(self, *args, **kwargs):
        r = self.radius
        l = self.length
        w = self.width
        jacobian = 1/r*np.array(
            [-l-w, l+w, l+w, -l-w,
              1, 1, 1, 1,
             -1, 1, -1, 1])
        return self.jacobian.format(jacobian)

    def to_joint(self, vel_2d):
        jb = self.jb().ndarray()
        u = np.transpose(jb) @ vel_2d.ndarray().reshape((3, 1))
        return u.flatten()


if __name__ == '__main__':
    m = MecanumDriveDef()
    m.define()
    pass
