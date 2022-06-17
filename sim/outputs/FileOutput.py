from sim.outputs.OutputBase import OutputBase
import pandas as pd


class FileOutput(OutputBase):
    def __init__(self, filepath):
        super().__init__()
        self.filepath = filepath

    def make_output(self):
        """make proper output from the data"""
        self._save_2_file()

    def _save_2_file(self):
        df = pd.DataFrame.from_dict(self._data)
        df.to_csv(self.filepath, index=False)
