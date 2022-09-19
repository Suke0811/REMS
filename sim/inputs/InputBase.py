from sim.typing import DefDict

class StdKeyMap:
    pass

SEPARATOR = '.'

class InputBase:
    def __init__(self, estop_callback=None):
        self._timestamps = []
        self.inpt = None
        self._command = None
        self._quit = False
        self._estop = False
        self.estop_callback = estop_callback

    def init(self, input_def=None):
        pass

    def get_inputs(self, timestamp=None, prefix='inpt', *args, **kwargs):
        """prefix specify what data to get, if noe"""
        if prefix in self.inpt.prefixes:
            ret_inpt = self.inpt.__dict__[prefix]()
        else:
            ret_inpt = self.inpt
        return ret_inpt

    def get_command(self):
        """For general key handling such as setting commands"""
        pass

    def if_exit(self):
        """if exit is pressed"""
        return self._quit

    @property
    def estop(self):    # read only
        return self._estop

    @property
    def quite(self):    # read only
        return self._quit

    def if_estop(self):
        """For E-stop command
        Ignite callback in the main?"""
        if self._estop and self.estop_callback is not None:
            self.estop_callback()
        return self._estop

    def _find_all_prefixes(self, data_list):
        prefix =[]
        for d in data_list:
            if d.find(SEPARATOR) >= 1:
                prefix.append(d.split(SEPARATOR)[0])
        return prefix

