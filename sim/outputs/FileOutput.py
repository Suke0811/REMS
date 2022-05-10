from sim.outputs.OutputSystem import OutputSystem
from sim.constants import DATA
from sim.formulation import *
import pandas as pd


class FileOutput(OutputSystem):
    def __init__(self, filepath):
        super().__init__()
        self._filepath = filepath

    def make_output(self):
        """make proper output from the data"""
        self._save_2_file()

    def _save_2_file(self):
        df = pd.DataFrame.from_dict(self._data)
        df.to_csv(self._filepath, index=False)
