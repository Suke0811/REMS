from defdict import DefDict
from rems.inputs import InputBase

class StdKeyMap:
    pass

SEPARATOR = '.'

class HockInput(InputBase):
    def __init__(self,  init_state=None, *args, **kwargs):
        super().__init__()
        self.init_state = init_state
        self.inpt = DefDict()

    def init(self, *args, **kwargs):
        pass

    def get_inputs(self, timestamp=None, prefix='inpt', *args, **kwargs):
        """prefix specify what data to get, if noe"""
        if prefix == 'state':
            ret_inpt = self.init_state
        else:
            ret_inpt = self.inpt
        return ret_inpt

