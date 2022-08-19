from sim.robots import RobotBase
from sim.device.connect.WifiDevice import WifiDevice
from sim.device.connect.WebsocketDevice import WebsocketDevice
from sim.device.state_estimator.ArucoDevice import ArucoDevice
from sim.utils import time_str

ID_LISTs = [2, 1]


class WoodbotHard(RobotBase):
    def __init__(self, ssid='ESP*', *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.run.DT = 0.2
        self.run.name = 'Web'
        self.add_device(WifiDevice, ssid=ssid)
        self.add_device(WebsocketDevice)
        #self.add_device(ArucoDevice, track_id=2, camera_id=2, video_name=f'video/aruco_{time_str()}.avi', dt=self.run.DT)

    def init(self, *args, **kwargs):
        super().init(*args, **kwargs)

    def drive(self, inpt, timestamp):
        inpt /= 5.24
        super().drive(inpt, timestamp)

