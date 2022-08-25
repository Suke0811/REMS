from sim.robots import RobotBase
from sim.device.Dynamixel.Dynamixel import Dynamixel
from sim.device.state_estimator.ArucoDevice import ArucoDevice
from sim.typing import DefDict
from sim.utils import time_str

ID_LISTs = [2, 1]


class DynamixelbotHard(RobotBase):
    def __init__(self, port, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.run.DT = 0.04
        self.run.name = 'Hard'
        self.add_device(Dynamixel, id_lists=ID_LISTs, slave_ids=None, device_port=port)
        self.add_device(ArucoDevice, track_id=2, camera_id=6, video_name=f'video/aruco_{time_str()}.avi', dt=self.run.DT)

    def init(self, *args, **kwargs):
        super().init(*args, **kwargs)

    def drive(self, inpt, timestamp):
        #inpt /= 3
        super(DynamixelbotHard, self).drive(inpt, timestamp)

