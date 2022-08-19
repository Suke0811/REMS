from sim.device import BasicDeviceBase


class DriveBase(BasicDeviceBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.config.on().set([True, False, False])

    def drive(self, inpt, timestamp, *args, **kwargs):
        pass

