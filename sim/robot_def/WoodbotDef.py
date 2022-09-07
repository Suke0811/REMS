from sim.robot_def.webots import DifferentialDriveDef
from sim.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType
# sensor names and definitions
SENSOR = {
    'lidar_f': float, 'lidar_r': float,
    'mag_x':float, 'mag_y': float,
    'gyro_z': float,
}

class WoodbotVel(AngVel):
    default_unit = 'rad/s'
    default_value = None
    default_dtype = float
    default_drange = (-7, 7)
    default_drange_map = None
    # data scale. (-1, 1) -> -100% to 100%. (0, 1) -> 0% to 100%
    defualt_drange_scale = (-1, 1)

# Driver names and definitions
# pos = inf means velocity control
DRIVE = {
    "motor_l": dict(pos=Ang(default=float('inf')), vel=WoodbotVel, acc=AngAcc, on=bool, pid=list),
    "motor_r": dict(pos=Ang(default=float('inf')), vel=WoodbotVel, acc=AngAcc, on=bool, pid=list),
}


class WoodbotDef(DifferentialDriveDef):
    def __init__(self, *args, **kwargs):
         super().__init__(radius=0.04, length=0.11, max_vel=1, *args, **kwargs)

    def define(self, *args, **kwargs):
        super().define(DRIVE, SENSOR)
        self.name = 'Woodbot'
        print(self.inpt.rules)


