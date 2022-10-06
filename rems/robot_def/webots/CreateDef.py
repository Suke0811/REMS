from rems.robot_def.webots import DifferentialDriveDef
from rems.robot_def.webots.DifferentialDriveDef import rule_to_nested_vel
from rems.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType, Percent
from rems.robots.differential_drive.CreateHard import CreateHard
from rems.robots.differential_drive.DynabotHard import DynabotHard
from rems.robots.differential_drive.WoodbotHard import WoodbotHard
from rems.typing import MapRule, DefDict

# sensor names and definitoins

OUTPT = {
"bumper_left": bool, "bumper_right": bool,
    "cliff_left": int, "cliff_front_left": int,
    "cliff_front_right": int, "cliff_right": int
}

ID_LISTs = [2, 1]

SENSOR = {"Webots": {
    "bumper_left": bool, "bumper_right": bool,
    "cliff_left": int, "cliff_front_left": int,
    "cliff_front_right": int, "cliff_right": int
},
    "Create2Device": CreateHard.sense_space_def().get("Create2Device"),
    "Dynamixel": DynabotHard.sense_space_def(IDs=ID_LISTs).get("Dynamixel"),
    "Woodbot": WoodbotHard.sense_space_def()
}

out_hard = [MapRule(['light_bumper_left', 'light_bumper_right'],
                      lambda *val: [round(v/4095) for v in val],    # hardware doesnt return 0-4095
                      {"bumper_left": bool, "bumper_right": bool},to_list=True),
              MapRule(['cliff_left_signal', 'cliff_front_left_signal', 'cliff_front_right_signal', 'cliff_right_signal'],
                      None,
                      {"cliff_left": int, "cliff_front_left": int, "cliff_front_right": int, "cliff_right": int}, to_list=True)
              ]

wb_drive = DefDict({
        "left wheel motor": dict(pos=float('inf'), vel=AngVel(drange=(-16, 16)), acc=AngAcc, on=bool),
        "right wheel motor": dict(pos=float('inf'), vel=AngVel(drange=(-16, 16)), acc=AngAcc, on=bool),
    })


DRIVE = {
    "Webots": wb_drive.set_rule(rule_to_nested_vel),
    "Create2Device": CreateHard.drive_space_def().get("Create2Device"),
    "Dynamixel": DynabotHard.drive_space_def(IDs=ID_LISTs).get('Dynamixel').set_rule(rule_to_nested_vel),
    "Woodbot": WoodbotHard.drive_space_def(),
}



class CreateDef(DifferentialDriveDef):
    def __init__(self, *args, **kwargs):
        super().__init__(radius=0.031, length=0.135878*2, *args, **kwargs)

    def define(self, *args, **kwargs):
        super().define(inpt_unit=AngVel(drange=(-16, 16)), drive_space=DRIVE, sense_space=SENSOR, *args, **kwargs)
        self.outpt.set_rule(out_hard)
        self.name = 'Create'
