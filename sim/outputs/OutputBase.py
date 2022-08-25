from sim.typing.DefDict import DefDict
import copy

class OutputBase:
    def __init__(self):
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
        self._timestamps.append({'timestamp': timestamp})
        self._inpts.append(copy.deepcopy(inpt))
        self._states.append(copy.deepcopy(state))
        self._outpts.append(copy.deepcopy(outpt))
        self._info.append(copy.deepcopy(info))

    def to_dict(self, data):
        "make all DefDict to dict format"
        return list(map(lambda d: {**dict(d)}, data))



    def make_output(self):
        """make proper output from the data"""
        raise NotImplementedError


