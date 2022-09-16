from sim.inputs import InputBase
import pandas as pd
import yaml
import json
from sim.typing.definitions import *
from sim.typing import DefDict


TIMESTAMP = 'timestamp'
DEFAULT_EXTENSION = 'yml'
SEPARATOR = '.'


class FileInput(InputBase):
    def __init__(self, filepath, loop=False):
        super().__init__()
        if filepath.find('.') == -1:
            filepath = filepath + '.' + DEFAULT_EXTENSION
        self._filepath = filepath
        self.file_extension = filepath.split('.')[-1]
        self.loop = loop
        self.timestamp_offset = 0.0
        # required call
        self.data = []
        self.set_data_from_df(self._open_file())
        self.definition = None
        self.inpt = None
        self.time_index = 0

    def get_inputs(self, timestamp=None, prefix='inpt', *args, **kwargs):
        """ """
        data = self._find_input_by_timestamp(timestamp - self.timestamp_offset)
        if prefix in data.prefixes:
            inpt_ret = data.__dict__[prefix]()
        else:
            inpt_ret = data
        print(inpt_ret)
        return inpt_ret

    def if_exit(self):
        return self._quit

    def _open_file(self):
        if self.file_extension == 'yml' or self.file_extension == 'yaml':
            with open(self._filepath, 'r') as f:
                df = pd.json_normalize(yaml.safe_load(f), max_level=1)
                df = df.rename(columns={TIMESTAMP+'.'+TIMESTAMP: TIMESTAMP})
        elif self.file_extension == 'json':
            with open(self._filepath, 'r') as f:
                df = pd.json_normalize(json.load(f), max_level=1)
                df = df.rename(columns={TIMESTAMP+'.'+TIMESTAMP: TIMESTAMP})
        elif self.file_extension == 'csv':
            df = pd.read_csv(self._filepath)
        else:
            raise ValueError(f'File extension should be yml, json, or csv, but {self.file_extension} was specified')
        return df

    def set_data_from_df(self, df):
        rowNames = df.columns
        df_dict = df.to_dict('list')
        prefix = self._find_all_prefixes(rowNames.to_list())
        self._inpt = DefDict(rowNames.to_list(), prefixes=prefix)
        self.data = list(zip(*(df_dict[k] for k in rowNames.to_list())))  # zipping 2 lists to create list of tuple
        self._timestamps = list(df_dict[TIMESTAMP])
        # initiate _inpt with the first data
        self._inpt.set(list(self.data[0]))

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



