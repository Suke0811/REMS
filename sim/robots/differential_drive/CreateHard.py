from sim.robots import RobotBase
from sim.device.iRobot.Create2Device import Create2Device
from sim.device.state_estimator.ArucoDevice import ArucoDevice
from sim.typing import DefDict
from sim.utils import time_str


class CreateHard(RobotBase):
    DEVICE_LIST = [Create2Device, ArucoDevice]
    def __init__(self, port, camera_id=6, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.run.DT = 0.1
        self.run.name = 'Hard'
        self.port = port
        self.camera_id = camera_id

    def init_devices(self):
        self.add_device(Create2Device(port=self.port))
        self.add_device(
            ArucoDevice(track_id=3, camera_id=self.camera_id, video_name=f'video/aruco_{time_str()}.avi',
                        dt=self.run.DT))

    def init(self, *args, **kwargs):
        super().init(*args, **kwargs)


