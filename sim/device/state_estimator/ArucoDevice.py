from sim.device.ObserveStateBase import ObserveStateBaseBasic
from sim.utils.ArucoHelper import ArucoHelper
from sim.typing import DefDict
from sim.utils import time_str
import numpy as np

ARUCO_STATE = dict(x=float, y=float, z=float, th_z=float, th_y=float, th_x=float)
VEL_2D = dict(d_x=float, d_y=float, d_th_y=float)

class ArucoDevice(ObserveStateBaseBasic):
    def __init__(self, track_id, camera_id=0, video_name=f'video/aruco_{time_str()}.avi', dt=0.1):
        super().__init__()
        self.track_id = track_id
        self.camera_id = camera_id
        self.vel = [0.0, 0.0, 0.0]
        self.to_thread = True
        self.video_name = video_name
        self.dt = dt

    def observe_state(self, *args, **kwargs):
        self._track_makers()
        return self.state

    def _track_makers(self):
        frames = self.aruco.get_frames([self.track_id])
        track_index = 0
        if not frames:
            return
        self.state.set(frames[track_index])
        # p_s = np.array(self.state[0:3])
        # dth_notUpdated = self.vel[2]
        # self.vel = list((s-p_s)/self.DT)
        # if self.vel[2] >= 2.0:
        #     self.vel[2] = dth_notUpdated

    def init(self):
        self.state = DefDict(ARUCO_STATE)
        self.aruco = ArucoHelper(fps=round(1 / self.dt), camera_id=self.camera_id, video_name=self.video_name)

    def open(self):
        self.aruco.init_camera()

    def close(self):
        self.aruco.close()
