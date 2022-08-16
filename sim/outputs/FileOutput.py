from sim.outputs.OutputBase import OutputBase
from pathlib import Path
import pandas as pd


class FileOutput(OutputBase):
    def __init__(self, filepath):
        super().__init__()
        self.filepath = filepath

    def make_output(self):
        """make proper output from the data"""
        self._save_2_file(self._timestamps, self._inpts, self._states, self._outpts, self._info)

    def _save_2_file(self, *data):
        dfs = []
        for d in data:
            dfs.append(pd.DataFrame.from_dict(self.to_dict(d)))
        df = pd.concat(dfs, axis=1)
        p = Path(self.filepath).parent.mkdir(parents=True, exist_ok=True)   # create dir if it doesn't exist
        df.to_csv(self.filepath, index=False)
