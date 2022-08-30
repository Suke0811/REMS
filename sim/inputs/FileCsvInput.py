from sim.inputs import InputBase
from sim.typing.definitions import *
from sim.typing import DefDict
import pandas as pd
from sim.utils import tictoc

SEPARATOR = '.'

class FileCsvInput(InputBase):
    def __init__(self, filepath, loop=False):
        super().__init__()
        self._filepath = filepath
        self.loop = loop
        self.timestamp_offset = 0.0
        # required call
        self.data = []
        self._open_file()
        self.definition = None
        self.inpt = None
        self.time_index = 0


    def get_inputs(self, inpt: DefDict, timestamp=None, prefix='inpt'):
        """ """
        data = self._find_input_by_timestamp(timestamp - self.timestamp_offset)
        if prefix in data.prefixes:
            inpt.set(data.__dict__[prefix]())
        else:
            inpt.set(data)
        return inpt

    def if_exit(self):
        return self._quit

    def _open_file(self):
        df = pd.read_csv(self._filepath)
        rowNames = df.columns
        df_dict = df.to_dict('list')
        prefix = self._find_all_prefixes(rowNames.to_list())
        self._inpt = DefDict(rowNames.to_list(), prefixes=prefix)
        self.data = list(zip(*(df_dict[k] for k in rowNames.to_list())))  # zipping 2 lists to create list of tuple
        self._timestamps = list(df_dict[TIMESTAMP])
        # initiate _inpt with the first data
        self._inpt.set(list(self.data[0]))

    def _find_all_prefixes(self, data_list):
        prefix =[]
        for d in data_list:
            if d.find(SEPARATOR) >= 1:
                prefix.append(d.split(SEPARATOR)[0])
        return prefix

    def _find_input_by_timestamp(self, timestamp):
        # Zero-order hold: return most recently specified inputs
        _timestamps = self._timestamps[self.time_index:-1]
        for t in _timestamps:
            if t > timestamp:
                self.time_index = self._timestamps.index(t)
                self._inpt.set(list(self.data[self.time_index]))
                return self._inpt


        if self.loop:
            self.timestamp_offset += timestamp
            self.time_index = 0
        else:
            self._quit = True
        return self._inpt

