from sim.typing import DefDict
from sim.device import BasicDeviceBase

class DeviceBase(BasicDeviceBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.config.on().set([True, True, True])

    def drive(self, inpt: DefDict, timestamp, *args, **kwargs):
        pass

    def sense(self, *args, **kwargs):
        pass

    def observe_state(self, *args, **kwargs):
        pass
