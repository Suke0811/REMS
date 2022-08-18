from sim.device import BasicDeviceBase


class ObserveStateBaseBasic(BasicDeviceBase):
    def __init__(self):
        super().__init__()
        self.config.on().set([False, False, True])

    def observe_state(self):
        pass
