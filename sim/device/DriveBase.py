from sim.device import BasicDeviceBase


class DriveBase(BasicDeviceBase):
    def __init__(self):
        super().__init__()
        self.config.on().set([True, False, False])

    def drive(self, inpt, timestamp):
        pass

