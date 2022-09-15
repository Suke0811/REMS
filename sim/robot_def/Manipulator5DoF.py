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

JOINT_VEL_SPACE = {'d_j.0': Percent(scale=(-1, 1)), 'd_j.1': Percent(scale=(-1, 1)), 'd_j.2': Percent(scale=(-1, 1)),
'd_j.3': Percent(scale=(-1, 1)), 'd_j.4': Percent(scale=(-1, 1)), 'd_j.5': Percent(scale=(-1, 1)),
               'd_g.0': Percent(scale=(-1, 1))
               }


JOINT_POS_SPACE = {'j.0': Percent(scale=(-1, 1)), 'j.1': Percent(scale=(-1, 1)), 'j.2': Percent(scale=(-1, 1)),
'j.3': Percent(scale=(-1, 1)), 'j.4': Percent(scale=(-1, 1)), 'j.5': Percent(scale=(-1, 1)),
               'g.0': Percent(scale=(-1, 1))
               }
J = dict(pos=Percent(scale=(-1, 1)), vel=Percent(scale=(-1, 1)))

JOINT_SPACE = {
    'j.0':J, 'j.1': J, 'j.2': J,
    'j.3': J, 'j.4': J, 'j.5': J,
    'g.0': J}



# Key mapping
class KeyMapRule:
    def __init__(self):
        self.joint_space = DefDict(JOINT_SPACE)
        self.arrow = MapRule(
            ['q', 'w', 'e', 'r', 't', 'y','u', 'a','s','d','f','g', 'h', 'j', 'space'],
            self.direct_drive, to_list=True)

    def get_rules(self):
        return [self.arrow]

    def joystick_drive(self):
        pass

    def direct_drive(self, q, w, e, r, t, y,u, a,s,d,f,g,h,j, space):
        ret = np.array([0, 0, 0, 0, 0, 0, 0])
        if q: ret += np.array([1, 0, 0,0,0,0,0])
        if a: ret += np.array([-1, 0, 0,0,0,0,0])
        if w: ret += np.array([0, 1, 0,0,0,0,0])
        if s: ret += np.array([0, -1, 0,0,0,0,0])
        if e: ret += np.array([0, 0, 1,0,0,0,0])
        if d: ret += np.array([0, 0, -1,0,0,0,0])
        if r: ret += np.array([0, 0, 0,1,0,0,0])
        if f: ret += np.array([0, 0, 0,-1,0,0,0])
        if t: ret += np.array([0, 0, 0,0,1,0,0])
        if g: ret += np.array([0, 0, 0,0,-1,0,0])
        if y: ret += np.array([0, 0, 0,0,0,1,0])
        if h: ret += np.array([0, 0, 0,0,0,-1,0])
        if u: ret += np.array([0, 0, 0,0,0,0,1])
        if j: ret += np.array([0, 0, 0,0,0,0,-1])
        self.joint_space.vel().set(100*ret)

        if space:
            self.joint_space.pos().set([0 for i in range(len(self.joint_space))])
        else:
            self.joint_space.pos().set([None for i in range(len(self.joint_space))])
        return self.joint_space



class Manipulator5DoF(RobotDefBase):
    def __init__(self, radius=0.01, length=0.1, width=0.1, *args, **kwargs):
        """init with a specific initial state (optional) """
        RobotDefBase.__init__(self, *args, **kwargs)
        self.radius = radius
        self.length = length
        self.width = width

    def define(self, joint_units, drive_space=None, sense_space=None, *args, **kwargs):
        """Definitions of the robot"""
        self.rule = KeyMapRule()
        self.inpt.add_def({k: DefDict(joint_units) for k in JOINT_SPACE.keys()},  rules=self.rule.arrow) # same definitino as input but with unit specified
        self.state.add_def(POS_2D)
        if drive_space is not None:
            self.drive_space.add_def(drive_space)
        if sense_space is not None:
            self.sense_space.add_def(sense_space)
        self.joint_space = None
        self.task_space = None
        super().define()

    # def drive(self, inpt, t, *args, **kwargs):
    #     inpt =
    #     super().drive(inpt, t, args, kwargs)


    def to_joint(self, vel_2d):
        jb = self.jb().ndarray()
        u = np.transpose(jb) @ vel_2d.ndarray().reshape((3, 1))
        return u.flatten()
