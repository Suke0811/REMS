from rems.robot_def.webots import DifferentialDriveDef
from rems.robot_def.webots.DifferentialDriveDef import rule_to_nested_vel
from rems.robots.differential_drive.CreateHard import CreateHard
from rems.robots.differential_drive.DynabotHard import DynabotHard
from rems.robots.differential_drive.WoodbotHard import WoodbotHard
from rems.typing import MapRule, DefDict
from rems.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType, Percent


OUTPT = {
    "so0": float, "so1": float, "so2": float, "so3": float, "so4": float, "so5": float, "so6": float, "so7": float,
    "so8": float, "so9": float, "so10": float, "so11": float, "so12": float,  "so13": float,  "so14": float, "so15": float,
}

ID_LISTs = [2, 1]

SENSOR = {"Webots": OUTPT,
    "Create2Device": CreateHard.sense_space_def().get("Create2Device"),
    "Dynamixel": DynabotHard.sense_space_def(IDs=ID_LISTs).get("Dynamixel"),
    "Woodbot": WoodbotHard.sense_space_def()
}

drange = (-5.24, 5.24)
wb_drive = DefDict({
    "left wheel": dict(pos=float('inf'), vel=AngVel(drange=drange), acc=AngAcc, on=bool),
    "right wheel": dict(pos=float('inf'), vel=AngVel(drange=drange), acc=AngAcc, on=bool),
})


DRIVE = {
    "Webots": wb_drive.set_rule(rule_to_nested_vel),
    "Create2Device": CreateHard.drive_space_def().get("Create2Device"),
    "Dynamixel": DynabotHard.drive_space_def(IDs=ID_LISTs).get('Dynamixel').set_rule(rule_to_nested_vel),
    "Woodbot": WoodbotHard.drive_space_def(),
}



class Pioneer3DxDef(DifferentialDriveDef):
    def __init__(self):
        super().__init__(radius=0.0975, length=0.165*2)

    def define(self, *args, **kwargs):
        super().define(inpt_unit=AngVel(drange=drange), drive_space=DRIVE, sense_space=SENSOR, *args, **kwargs)
        self.name = 'Pioneer 3-DX'

