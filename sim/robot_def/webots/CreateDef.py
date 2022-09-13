from sim.robot_def.webots import DifferentialDriveDef
from sim.robot_def.webots.DifferentialDriveDef import rule_to_nested_vel
from sim.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType, Percent
from sim.robots.differential_drive.CreateHard import CreateHard
from sim.robots.differential_drive.DynabotHard import DynabotHard
from sim.typing import MapRule, DefDict

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
    "Woodbot": bool,
}

inpt = {'wh.l': AngVel(drange=(-16, 16)), 'wh.r': AngVel(drange=(-16, 16))}
# inpt = {'wh.l': Percent(scale=(-1, 1)), 'wh.r': Percent(scale=(-1, 1))}


class CreateDef(DifferentialDriveDef):
    def __init__(self, *args, **kwargs):
        super().__init__(radius=0.031, length=0.135878*2, *args, **kwargs)

    def define(self, *args, **kwargs):
        super().define(inpt, joint_unit=AngVel(drange=(-16, 16)))
        self.outpt.add_def(OUTPT).set_rule(out_hard)
        self.drive_space.add_def(DRIVE)
        self.sense_space.add_def(SENSOR)
        self.name = 'Create'
