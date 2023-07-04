from rems.robot_def import MecanumDriveDef
from defdict.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType, Percent
from defdict import MapRule, DefDict

# sensor names and definitoins

OUTPT = {
}

ID_LISTs = [2, 1]

SENSOR = {"Webots": OUTPT}

wb_drive = DefDict({
    "wheel2": dict(pos=float('inf'), vel=AngVel(drange=(-14.8, 14.8)), acc=AngAcc, on=bool),
    "wheel1": dict(pos=float('inf'), vel=AngVel(drange=(-14.8, 14.8)), acc=AngAcc, on=bool),
    "wheel3": dict(pos=float('inf'), vel=AngVel(drange=(-14.8, 14.8)), acc=AngAcc, on=bool),
    "wheel4": dict(pos=float('inf'), vel=AngVel(drange=(-14.8, 14.8)), acc=AngAcc, on=bool),
    })

DRIVE = {
    "Webots": wb_drive,
}

WHEEEL_VEL = {'wh.0': AngVel(drange=(-14.8, 14.8)), 'wh.1': AngVel(drange=(-14.8, 14.8)),
              'wh.2': AngVel(drange=(-14.8, 14.8)), 'wh.3': AngVel(drange=(-14.8, 14.8))}

ARROW_VEL = {'th_z': AngVel(drange=(-0.5, 0.5)), 'd_x': Vel(drange=(-0.5, 0.5)), 'd_y': Vel(drange=(-0.5, 0.5))}

class YoubotBaseDef(MecanumDriveDef):
    def __init__(self, *args, **kwargs):
        MecanumDriveDef.__init__(self, radius=0.05, length=0.228, width=0.158, *args, **kwargs)

    def define(self, *args, **kwargs):
        MecanumDriveDef.define(self, arrow_units=ARROW_VEL, inpt_unit=AngVel(drange=(-14.8, 14.8)), drive_space=DRIVE, sense_space=SENSOR, *args, **kwargs)
        self.drive_space['Webots'].set_rule(MapRule(['th_z', 'd_x', 'd_y'],
                             self._set_vel, ['wheel2', 'wheel1', 'wheel3', 'wheel4'],
                             with_target=True))
        self.name = 'youBotBase'

    def _set_vel(self, o, t):
        joint = self.to_joint(o)
        t.vel().set(joint)
        t.pos().set({k: float('inf') for k, v in WHEEEL_VEL.items()})
        return t
