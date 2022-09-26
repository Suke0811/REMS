from rems.robot_def.webots import DifferentialDriveDef
from rems.robot_def.webots.DifferentialDriveDef import rule_to_nested_vel
from rems.robots.differential_drive.CreateHard import CreateHard
from rems.robots.differential_drive.DynabotHard import DynabotHard
from rems.robots.differential_drive.WoodbotHard import WoodbotHard
from rems.typing import MapRule, DefDict
from rems.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType, Percent

OUTPT = {
    "accelerometer": dict(x=float, y=float, z=float),
    "gyro": dict(x=float, y=float, z=float),
    "ps0": float, "ps1": float, "ps2": float, "ps3": float, "ps4": float, "ps5": float, "ps6": float, "ps7": float,
    "ls0": float, "ls1": float, "ls2": float, "ls3": float, "ls4": float, "ls5": float, "ls6": float, "ls7": float,
}

ID_LISTs = [2, 1]
SENSOR = {"Webots": OUTPT,
    "Create2Device": CreateHard.sense_space_def().get("Create2Device"),
    "Dynamixel": DynabotHard.sense_space_def(IDs=ID_LISTs).get("Dynamixel"),
    "Woodbot": WoodbotHard.sense_space_def()
}


drange=(-6.28, 6.28)
wb_drive = DefDict({
        "left wheel motor": dict(pos=float('inf'), vel=AngVel(drange=drange), acc=AngAcc, on=bool),
        "right wheel motor": dict(pos=float('inf'), vel=AngVel(drange=drange), acc=AngAcc, on=bool),
    })



DRIVE = {
    "Webots": wb_drive.set_rule(rule_to_nested_vel),
    "Create2Device": CreateHard.drive_space_def().get("Create2Device"),
    "Dynamixel": DynabotHard.drive_space_def(IDs=ID_LISTs).get('Dynamixel').set_rule(rule_to_nested_vel),
    "Woodbot": WoodbotHard.drive_space_def(),
}


class EpuckDef(DifferentialDriveDef):
    def __init__(self):
        super().__init__(radius=0.02, length=0.052)

    def define(self, *args, **kwargs):
        super().define(inpt_unit=AngVel(drange=drange), drive_space=DRIVE, sense_space=SENSOR, *args, **kwargs)
        self.name = 'e-puck'

