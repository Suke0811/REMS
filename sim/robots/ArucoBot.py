from sim.robots.RobotBase import RobotBase
from sim.device.state_estimator.ArucoDevice import ArucoDevice
from sim.utils import time_str



class ArucoBot(RobotBase):
    DEVICE_LIST = [ArucoDevice]
    def __init__(self, tracids=None, camera_id=0, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.run.DT = 0.05
        self.run.name = 'Aruco'
        self.run.supress_info = True
        self.tracids = tracids
        self.camera_id = camera_id
        self.aruco = ArucoDevice(track_id=self.tracids, camera_id=self.camera_id, video_name=f'video/aruco_{time_str()}.avi',
                        dt=self.run.DT)
        self.add_device(self.aruco)

    def define(self, *args, **kwargs):
        self.state = self.aruco.all_data
        super().define()



