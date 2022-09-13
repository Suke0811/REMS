from sim.typing import DefDict

class BasicDeviceBase:
    TO_THREAD = 'to_thread'
    TO_PROCESS = 'to_process'
    device_name = 'device'
    def __init__(self, *args, **kwargs):
        self.to_thread = None
        self.config = DefDict(dict(drive=dict(on=False, step=float), sense=dict(on=False, step=float), observe_state=dict(on=False, step=float)),)

    def init(self, *args, **kwargs):
        pass

    def open(self, *args, **kwargs):
        pass

    def close(self, *args, **kwargs):
        pass

    def enable(self, enable, *args, **kwargs):
        pass

