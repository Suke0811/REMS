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

    def set_drive_space(self, drive_space=None):
        if drive_space is None:
            drive_space = self.create_drive_space
        self.drive_space = drive_space

    @staticmethod
    def create_drive_space(*args, **kwargs):
        return DefDict()

    def set_sense_space(self, sense_space=None):
        if sense_space is None:
            sense_space = self.create_sense_space()
        self.sense_space = sense_space

    @staticmethod
    def create_sense_space(*args, **kwargs):
        return DefDict()


