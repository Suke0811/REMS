from sim.robots import RobotBase
from sim.device.Dynamixel.Dynamixel import Dynamixel
from sim.device.state_estimator.ArucoDevice import ArucoDevice
from sim.typing import DefDict
from sim.utils import time_str

ID_LISTs = [2, 1]



class DynabotHard(RobotBase):
    DEVICE_LIST = [Dynamixel, ArucoDevice]
    def __init__(self, port, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.run.DT = 0.05
        self.run.name = 'Hard'
        self.port = port

    def init_devices(self):
        self.add_device(Dynamixel(ID_LISTs, device_port=self.port))

    def init(self, *args, **kwargs):
        super().init(*args, **kwargs)




