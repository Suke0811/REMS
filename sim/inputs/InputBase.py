from sim.typing import DefDict

class StdKeyMap:
    pass


class InputBase:
    def __init__(self, estop_callback=None):
        self._timestamps = []
        self._inputs = None
        self._command = None
        self._quit = False
        self._estop = False
        self.estop_callback = estop_callback

    def init(self):
        pass

    def get_inputs(self, inpt_def: DefDict, timestamp=None):
        inpt_def.set(self._inputs)
        return inpt_def

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

