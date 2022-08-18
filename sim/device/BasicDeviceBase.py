from sim.typing import DefDict


class BasicDeviceBase:
    def __init__(self):
        self.to_thread = False
        self.device_name = None
        self.config = DefDict(dict(drive=dict(on=False), sense=dict(on=False), observe_state=dict(on=False)),
                              suffixes=['on'])

    def init(self):
        pass

    def open(self):
        pass

    def close(self):
        pass

    def enable(self, enable):
        pass

