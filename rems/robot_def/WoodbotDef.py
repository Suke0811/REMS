from rems.robot_def.webots import DifferentialDriveDef
from rems.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType, Percent
from rems.robots.differential_drive.CreateHard import CreateHard
from rems.robots.differential_drive.DynabotHard import DynabotHard
from rems.robots.differential_drive.WoodbotHard import WoodbotHard
from rems.typing import MapRule, DefDict
# sensor names and definitions

OUTPUT = {
    'lidar_f': float, 'lidar_r': float,
    'mag_x':float, 'mag_y': float,
    'gyro_z': float,
}

class WoodbotVel(AngVel):
    default_unit = 'rad/s'
    default_value = None
    default_dtype = float
    default_drange = (-8, 8)
    default_drange_map = None
    # data scale. (-1, 1) -> -100% to 100%. (0, 1) -> 0% to 100%
    defualt_drange_scale = (-1, 1)


ID_LISTs = [2, 1]

SENSOR = {"Webots": OUTPUT,
    "Create2Device": CreateHard.sense_space_def().get("Create2Device"),
    "Dynamixel": DynabotHard.sense_space_def(IDs=ID_LISTs).get("Dynamixel"),
    "Woodbot": WoodbotHard.sense_space_def()
}


wb_drive = DefDict({
    "motor_l": dict(pos=Ang(default=float('inf')), vel=WoodbotVel, acc=AngAcc, on=bool, pid=list),
    "motor_r": dict(pos=Ang(default=float('inf')), vel=WoodbotVel, acc=AngAcc, on=bool, pid=list),
})

def set_vel(o, t):
    t.vel().set_positional(o*-1)
    t.pos().set([float('inf'), float('inf')])
    return t

rule_to_nested_vel = MapRule(['wh.l', 'wh.r'],
            set_vel,
            ['motor_r', 'motor_l'],
            with_target=True)


DRIVE = {
    "Webots": wb_drive.set_rule(rule_to_nested_vel),
    "Create2Device": CreateHard.drive_space_def().get("Create2Device"),
    "Dynamixel": DynabotHard.drive_space_def(IDs=ID_LISTs).get('Dynamixel').set_rule(rule_to_nested_vel),
    "Woodbot": WoodbotHard.drive_space_def(),
    #"model":
}


class WoodbotDef(DifferentialDriveDef):
    def __init__(self, *args, **kwargs):
         super().__init__(radius=0.02, length=0.11, *args, **kwargs)

    def define(self, *args, **kwargs):
        super().define(WoodbotVel, DRIVE, SENSOR)
        self.outpt.add_def(OUTPUT)
        self.name = 'woodbot'
        print(self.inpt.rules)


