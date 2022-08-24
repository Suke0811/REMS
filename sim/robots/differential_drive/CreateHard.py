from sim.robots import RobotBase
from sim.device.iRobot.Create2Device import Create2Device
from sim.device.state_estimator.ArucoDevice import ArucoDevice
from sim.typing import DefDict
from sim.utils import time_str


class CreateHard(RobotBase):
    def __init__(self, port, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.run.DT = 0.1
        self.run.name = 'Hard'
        self.add_device(Create2Device, port=port)
        self.add_device(ArucoDevice, track_id=3, camera_id=2, video_name=f'video/aruco_{time_str()}.avi', dt=self.run.DT)

    def init(self, *args, **kwargs):
        super().init(*args, **kwargs)

