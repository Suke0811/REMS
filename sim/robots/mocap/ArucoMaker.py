from sim.modules.RobotSystem import *
from sim.utils.ArucoHelper import ArucoHelper
from sim.formulation import STATE_SPACE
from sim.constants import TEST

class ArucoMaker(RobotSystem):
    def __init__(self, track_id, display=False, camera_id=0):
        super().__init__()
        #aruco, You need to change CALIBRATION_PATH
        self.aruco = ArucoHelper(fps=int(1/TEST.DT), camera_id=camera_id)
        self.aruco.init_camera()
        self.track_id = track_id
        # sim settings
        self.realtime = True
        self.to_thread = not display

    def drive(self, inpts, timestamp):
        """drive the robot to the next state
        :param inpts: left, right wheel velocities
        :return full state feedback"""
        self._track_makers()
        return self.state

    def _track_makers(self):
        frames = self.aruco.get_frames([self.track_id])
        track_index = 0
        if not frames:
            return
        state = [frames[track_index][0], frames[track_index][1], frames[track_index][3], 0.0]
        self.state = [float(state[i]) for i in range(len(STATE_SPACE))]

    def sense(self):
        return [0.0 for k in OUTPUT_SPACE]
