from rems.device import BasicDeviceBase


class ObserveStateBaseBasic(BasicDeviceBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.config.on().set([False, False, True])

    def observe_state(self, *args, **kwargs):
        pass
