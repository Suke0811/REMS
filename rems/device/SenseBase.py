from rems.device import BasicDeviceBase
from rems.typing import DefDict

class SenseBase(BasicDeviceBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.sense_space = None
        self.config.on().set(dict(sense=True))

    def sense(self, *args, **kwargs):
        pass

    def set_sense_space(self, sense_space=None):
        if sense_space is None:
            sense_space = self.create_sense_space()
        self.sense_space = sense_space

    @staticmethod
    def create_sense_space(*args, **kwargs):
        return DefDict()

