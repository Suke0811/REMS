from sim.typing.DefDict import DefDict


class OutputBase:
    def __init__(self):
        self._data = []
        self._states = []
        self._outpts = []
        self._inpts = []
        self._info = []
        self._timestamps = []

    def process(self, state, inpt, outpt, timestamp, info):
        """save data to dynamic memory
        :param state: state space
        :param inpt: input space
        :param outpt: output space
        :param timestamp: corresponding timestamp
        """
        self._timestamps.append(timestamp)
        self._inpts.append(inpt)
        self._states.append(state)
        self._outpts.append(outpt)
        self._info.append(info)
        d = {'timestamp': timestamp, **self.to_dict(inpt), **self.to_dict(state), **self.to_dict(outpt), **self.to_dict(info)}
        self._data.append(d)

    def to_dict(self, data):
        ret = {}
        if isinstance(data, DefDict):
            ret = data.get()
        return ret

    def make_output(self):
        """make proper output from the data"""
        raise NotImplementedError


