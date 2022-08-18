from sim.device import BasicDeviceBase


class SenseBase(BasicDeviceBase):
    def __init__(self):
        super().__init__()
        self.config.on().set([False, True, True])

    def sense(self):
        pass

    def observe_state(self):
        pass
