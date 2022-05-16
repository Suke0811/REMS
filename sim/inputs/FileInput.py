from sim.inputs import InputBase
from sim.type.definitions import *
from sim.type import DefDict
import pandas as pd


class FileInput(InputBase):
    def __init__(self, filepath, loop=False):
        super().__init__()
        self._filepath = filepath
        self.loop = loop
        self.timestamp_offset = 0.0
        # required call
        self.data = []
        self._open_file()
        self.definition = None

    def get_inputs(self, timestamp=None):
        return self._find_input_by_timestamp(timestamp-self.timestamp_offset)

    def if_exit(self):
        return self._quit

    def _open_file(self):
        df = pd.read_csv(self._filepath)
        rowNames = df.columns
        df_dict = df.to_dict('list')
        self._inpts = []
        for l in df_dict:
            dd = DefDict(rowNames).data_as(l)
            if dd is not None:
                self.data.append(dd)
                self._inpts.append(self.input_def.set_data(dd))
        self._timestamps = df_dict[TIMESTAMP]

    def _find_input_by_timestamp(self, timestamp):
        # Zero-order hold: return most recently specified inputs
        inpt = self._inpts[0]
        for t, i in zip(self._timestamps, self._inpts):
            if t > timestamp:
                return inpt
            inpt = i

        if self.loop:
            self.timestamp_offset += timestamp
        else:
            self._quit = True
        return inpt


if __name__ == '__main__':
    i = FileInput('test')
