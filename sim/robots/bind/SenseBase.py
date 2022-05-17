from sim.robots.bind import BasicDeviceBase


class SenseBaseBasic(BasicDeviceBase):
    def __init__(self):
        super().__init__()
        self.sensors = {}

    def sense(self, inpt, timestamp):
        pass

    def observe_state(self):
        pass
