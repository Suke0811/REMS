from sim.robots.webots import DifferentialDriveDef
from sim.typing import DefDict
import numpy as np

SENSOR = {
    "Sick LMS 291": list,
    "so0": float, "so1": float, "so2": float, "so3": float, "so4": float, "so5": float, "so6": float, "so7": float,
    "so8": float, "so9": float, "so10": float, "so11": float, "so12": float,  "so13": float,  "so14": float, "so15": float,
}

DRIVE = {
    "front left wheel": dict(pos=float('inf'), vel=float, acc=float, on=bool, pid=list),
    "front right wheel": dict(pos=float('inf'), vel=float, acc=float, on=bool, pid=list)
}


class Pioneer3AtDef(DifferentialDriveDef):
    def __init__(self):
        super().__init__(radius=0.0975, length=0.165*2, max_vel=6.4)

    def define(self, *args, **kwargs):
        super().define(DRIVE, SENSOR)
        self.run.name = 'Pioneer 3-AT'

