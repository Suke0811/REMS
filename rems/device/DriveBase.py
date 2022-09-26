from rems.device import BasicDeviceBase
from rems.typing import DefDict

class DriveBase(BasicDeviceBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.config.on().set([True, False, False])

    def drive(self, inpt, timestamp, *args, **kwargs):
        pass

    def set_drive_space(self, drive_space=None):
        if drive_space is None:
            drive_space = self.create_drive_space
        self.drive_space = drive_space

    @staticmethod
    def create_drive_space(*args, **kwargs):
        return DefDict()
