from sim.device import BasicDeviceBase


class SenseBase(BasicDeviceBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.config.on().set([False, True, True])

    def sense(self, *args, **kwargs):
        pass

    def observe_state(self, *args, **kwargs):
        pass
