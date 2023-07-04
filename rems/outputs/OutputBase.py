import copy

class OutputBase:
    def __init__(self):
        self._states = []
        self._outpts = []
        self._inpts = []
        self._info = []
        self._timestamps = []

    def init(self, *args, **kwargs):
        pass

    def process(self, state, inpt, outpt, timestamp, info):
        """save data to dynamic memory
        :param state: state space
        :param inpt: input space
        :param outpt: output space
        :param timestamp: corresponding timestamp
        """
        self._timestamps.append({'timestamp': timestamp})
        self._inpts.append(copy.deepcopy(inpt))
        self._states.append(copy.deepcopy(state))
        self._outpts.append(copy.deepcopy(outpt))
        self._info.append(copy.deepcopy(info))

    def make_output(self):
        """make proper output from the data"""
        pass

    def to_dict(self, data, dtype=None):
        "make all DefDict to dict format"
        if dtype is not None:
            if dtype == 'float':
                try:
                    return list(map(lambda d: {**dict(d.to_float().dict())}, data))
                except AttributeError:
                    pass
            elif dtype == 'int':
                try:
                    return list(map(lambda d: {**dict(d.to_int().dict())}, data))
                except AttributeError:
                    pass
        return list(map(lambda d: {**dict(d)}, data))

    def to_timeseries_dict(self):
        timeseries = []
        for t, state, inpt, outpt, info in zip(self.to_dict(self._timestamps, 'float'),
                                                       self.to_dict(self._states, 'float'), self.to_dict(self._inpts, 'float'),
                                                       self.to_dict(self._outpts, 'float'), self.to_dict(self._info, 'float')):
            d = dict(timestamp=t, inpt=inpt, state=state, outpt=outpt, info=info)
            timeseries.append(d)
        return timeseries

