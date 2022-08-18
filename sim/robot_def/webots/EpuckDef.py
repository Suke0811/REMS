from sim.robot_def.webots import DifferentialDriveDef

SENSOR = {
    "accelerometer": dict(x=float, y=float, z=float),
    "gyro": dict(x=float, y=float, z=float),
    "ps0": float, "ps1": float, "ps2": float, "ps3": float, "ps4": float, "ps5": float, "ps6": float, "ps7": float,
    "ls0": float, "ls1": float, "ls2": float, "ls3": float, "ls4": float, "ls5": float, "ls6": float, "ls7": float,
}

DRIVE = {
    "left wheel motor": dict(pos=float('inf'), vel=float, acc=float, on=bool, pid=list),
    "right wheel motor": dict(pos=float('inf'), vel=float, acc=float, on=bool, pid=list)
}


class EpuckDef(DifferentialDriveDef):
    def __init__(self):
        super().__init__(radius=0.02, length=0.052, max_vel=6.28)

    def define(self, *args, **kwargs):
        super().define(DRIVE, SENSOR)
        self.name = 'e-puck'

