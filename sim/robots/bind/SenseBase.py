from sim.robots.bind import DeviceBase


class SenseBase(DeviceBase):
    def __init__(self):
        super().__init__()
        self.sensors = {}

    def sense(self, inpt, timestamp, definition=None):
        pass

    def observe_state(self):
        pass
