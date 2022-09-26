from rems.robots import RobotBase
from rems.device.iRobot.Create2Device import Create2Device
from rems.typing import DefDict
from rems.utils import time_str


class CreateHard(RobotBase):
    DEVICE_LIST = [Create2Device]
    def __init__(self, port, camera_id=0, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.run.DT = 0.05
        self.run.name = 'Hard'
        self.port = port
        self.camera_id = camera_id

    def init_devices(self):
        self.add_device(Create2Device(port=self.port))


    def init(self, *args, **kwargs):
        super().init(*args, **kwargs)


