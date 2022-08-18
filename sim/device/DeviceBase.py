from sim.typing import DefDict
from sim.device import BasicDeviceBase

class DeviceBase(BasicDeviceBase):
    def __init__(self):
        super().__init__()
        self.config.on().set([True, True, True])

    def drive(self, inpt: DefDict, timestamp):
        pass

    def sense(self):
        pass

    def observe_state(self):
        pass
