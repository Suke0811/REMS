from sim.robots.RobotBase import RobotBase
from sim.robots.bind import DeviceBase

class FullDeviceBase(DeviceBase, RobotBase):
    def __init__(self):
        super(DeviceBase, self).__init__()
        super(FullDeviceBase, self).__init__()

    def init(self):
        pass

    def open(self):
        pass

    def close(self):
        pass

    def enable(self, enable):
        pass

    def drive(self, inpt, timestamp):
        pass

    def sense(self, definition=None):
        pass

    def observe_state(self, definition=None):
        return self.state

    def __del__(self):
        self.enable(False)
        self.close()
