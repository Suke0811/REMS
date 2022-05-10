from sim.type import DefDict

class StdKeyMap:
    pass


class InputBase:
    def __init__(self, input_definition=None, estop_callback=None):
        self._timestamps = []
        self._inputs = None
        self.input_def = input_definition
        self._command = None
        self._quit = False
        self._estop = False
        self.estop_callback = estop_callback

    def init(self, input_def: DefDict=None):
        if input_def is not None:
            self.input_def = input_def

        if self.input_def is None or isinstance(self.input_def, DefDict):
            raise ImportError('Key input definition must be defined')


    def get_input(self):
        self._inputs = self.input_def
        return self._inputs

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

