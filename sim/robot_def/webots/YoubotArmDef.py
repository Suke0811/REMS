from sim.robot_def.webots import DifferentialDriveDef
from sim.robot_def.webots.DifferentialDriveDef import rule_to_nested_vel
from sim.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType, Percent
from sim.robots.differential_drive.CreateHard import CreateHard
from sim.robots.differential_drive.DynabotHard import DynabotHard
from sim.robots.differential_drive.WoodbotHard import WoodbotHard
from sim.typing import MapRule, DefDict

# sensor names and definitoins

OUTPT = {
"bumper_left": bool, "bumper_right": bool,
    "cliff_left": int, "cliff_front_left": int,
    "cliff_front_right": int, "cliff_right": int
}

ID_LISTs = [2, 1]

SENSOR = {"Webots": OUTPT}


wb_drive = DefDict({
        "left wheel motor": dict(pos=float('inf'), vel=AngVel(drange=(-16, 16)), acc=AngAcc, on=bool),
        "right wheel motor": dict(pos=float('inf'), vel=AngVel(drange=(-16, 16)), acc=AngAcc, on=bool),
    })


DRIVE = {
    "Webots": wb_drive.set_rule(rule_to_nested_vel),
}



class YoubotArmDef(DifferentialDriveDef):
    def __init__(self, *args, **kwargs):
        super().__init__(radius=0.031, length=0.135878*2, *args, **kwargs)

    def define(self, *args, **kwargs):
        super().define(inpt_unit=AngVel(drange=(-16, 16)), drive_space=DRIVE, sense_space=SENSOR, *args, **kwargs)
        self.outpt.set_rule(out_hard)
        self.name = 'Create'
