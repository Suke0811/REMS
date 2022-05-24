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
        self.inpt = None


    def get_inputs(self, inpt: DefDict, timestamp=None):
        inpt.set_data(self._find_input_by_timestamp(timestamp-self.timestamp_offset))
        return inpt

    def if_exit(self):
        return self._quit

    def _open_file(self):
        df = pd.read_csv(self._filepath)
        rowNames = df.columns
        df_dict = df.to_dict('list')
        self._inpt = DefDict(rowNames.to_list())
        self.data = list(zip(*(df_dict[k] for k in rowNames.to_list())))  # zipping 2 lists to create list of tuple
        self._timestamps = df_dict[TIMESTAMP]

    def _find_input_by_timestamp(self, timestamp):
        # Zero-order hold: return most recently specified inputs
        self._inpt.data = list(self.data[0])
        for t, i in zip(self._timestamps, self.data):
            if t > timestamp:
                return self._inpt
            self._inpt.data = list(i)

        if self.loop:
            self.timestamp_offset += timestamp
        else:
            self._quit = True
        return self._inpt


if __name__ == '__main__':
    i = FileInput('test')
