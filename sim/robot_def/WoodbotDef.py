from sim.robot_def.webots import DifferentialDriveDef
# sensor names and definitoins
SENSOR = {

}

# Driver names and definitions
# pos = inf means velocity control
DRIVE = {
    "wh_l": dict(pos=float('inf'), vel=float, acc=float, on=bool, pid=list),
    "wh_r": dict(pos=float('inf'), vel=float, acc=float, on=bool, pid=list),
}


class WoodbotDef(DifferentialDriveDef):
    def __init__(self, *args, **kwargs):
         super().__init__(radius=0.04, length=0.11, max_vel=1, *args, **kwargs)

    def define(self, *args, **kwargs):
        super().define(DRIVE, SENSOR)
        self.name = 'Woodbot'


