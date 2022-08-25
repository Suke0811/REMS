from sim.typing import DefDict


class BasicDeviceBase:
    def __init__(self, *args, **kwargs):
        self.to_thread = False
        self.device_name = None
        self.config = DefDict(dict(drive=dict(on=False), sense=dict(on=False), observe_state=dict(on=False)),
                              suffixes=['on'])

    def init(self, *args, **kwargs):
        pass

    def open(self, *args, **kwargs):
        pass

    def close(self, *args, **kwargs):
        pass

    def enable(self, enable, *args, **kwargs):
        pass

