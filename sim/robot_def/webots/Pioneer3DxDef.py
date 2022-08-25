from sim.robot_def.webots import DifferentialDriveDef

SENSOR = {
    "so0": float, "so1": float, "so2": float, "so3": float, "so4": float, "so5": float, "so6": float, "so7": float,
    "so8": float, "so9": float, "so10": float, "so11": float, "so12": float,  "so13": float,  "so14": float, "so15": float,
}

DRIVE = {
    "left wheel": dict(pos=float('inf'), vel=float, acc=float, on=bool, pid=list),
    "right wheel": dict(pos=float('inf'), vel=float, acc=float, on=bool, pid=list)
}


class Pioneer3DxDef(DifferentialDriveDef):
    def __init__(self):
        super().__init__(radius=0.0975, length=0.165*2, max_vel=5.24)

    def define(self, *args, **kwargs):
        super().define(DRIVE, SENSOR)
        self.name = 'Pioneer 3-DX'

