from sim.bind import BasicDeviceBase


class SenseBase(BasicDeviceBase):
    def __init__(self):
        super().__init__()
        self.sensors = {}

    def sense(self, inpt, timestamp):
        pass

    def observe_state(self):
        pass
