from rems.robot_def.webots import DifferentialDriveDef
from rems.robot_def.webots.DifferentialDriveDef import rule_to_nested_vel
from rems.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType, Percent
from rems.robots.differential_drive.CreateHard import CreateHard
from rems.robots.differential_drive.DynabotHard import DynabotHard
from rems.robots.differential_drive.WoodbotHard import WoodbotHard
from rems.typing import MapRule, DefDict

# sensor names and definitoins

OUTPT = {
"compass": dict(x=float, y=float, z=float), "gps": dict(x=Pos, y=Pos, z=Pos),
}

ID_LISTs = [2, 1]

SENSOR = {"Webots": {},
    "Create2Device": CreateHard.sense_space_def().get("Create2Device"),
    "Dynamixel": DynabotHard.sense_space_def(IDs=ID_LISTs).get("Dynamixel"),
    "Woodbot": WoodbotHard.sense_space_def()
}


drange = (-26, 26)
wb_drive = DefDict({
        "left motor 1": dict(pos=float('inf'), vel=AngVel(drange=drange), acc=AngAcc, on=bool),
        "right motor 1": dict(pos=float('inf'), vel=AngVel(drange=drange), acc=AngAcc, on=bool),
        "left motor 2": dict(pos=float('inf'), vel=AngVel(drange=drange), acc=AngAcc, on=bool),
        "right motor 2": dict(pos=float('inf'), vel=AngVel(drange=drange), acc=AngAcc, on=bool),
        "left motor 3": dict(pos=float('inf'), vel=AngVel(drange=drange), acc=AngAcc, on=bool),
        "right motor 3": dict(pos=float('inf'), vel=AngVel(drange=drange), acc=AngAcc, on=bool),
        "left motor 4": dict(pos=float('inf'), vel=AngVel(drange=drange), acc=AngAcc, on=bool),
        "right motor 4": dict(pos=float('inf'), vel=AngVel(drange=drange), acc=AngAcc, on=bool),
        })

def set_vel(o, t):
    t.vel().set_positional(o)
    t.vel().filter(["left motor 2", "right motor 2"]).set_positional(o)
    t.vel().filter(["left motor 3", "right motor 3"]).set_positional(o)
    t.vel().filter(["left motor 4", "right motor 4"]).set_positional(o)
    t.pos().set([float('inf') for i in range(len(wb_drive))])
    return t

eight_wheel = MapRule(['wh.l', 'wh.r'],
            set_vel,
            with_target=True)

DRIVE = {
    "Webots": wb_drive.set_rule(eight_wheel),
    "Create2Device": CreateHard.drive_space_def().get("Create2Device"),
    "Dynamixel": DynabotHard.drive_space_def(IDs=ID_LISTs).get('Dynamixel').set_rule(rule_to_nested_vel),
    "Woodbot": WoodbotHard.drive_space_def(),
}


class MooseDef(DifferentialDriveDef):
    def __init__(self, *args, **kwargs):
        super().__init__(radius=0.31, length=1.9, *args, **kwargs)

    def define(self, *args, **kwargs):
        super().define(inpt_unit=AngVel(drange=drange), drive_space=DRIVE, sense_space=SENSOR, *args, **kwargs)
        self.name = 'moose'
