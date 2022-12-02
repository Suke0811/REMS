from rems.outputs.OutputBase import OutputBase
from rems.typing import DefDict
from pathlib import Path
import pandas as pd


class FileCsvOutput(OutputBase):
    def __init__(self, filepath):
        super().__init__()
        self.filepath = filepath

    def make_output(self):
        """make proper output from the data"""
        self._save_2_file(self._timestamps, self._inpts, self._states, self._outpts, self._info)

    def _save_2_file(self, *data):
        dfs = []
        for d in data:
            prefix = ''
            if d and isinstance(d[0], DefDict) and d[0].name is not None:
                prefix = d[0].name + '.'
            dfs.append(pd.DataFrame.from_dict(self.to_dict(d)).add_prefix(prefix))
        df = pd.concat(dfs, axis=1)
        p = Path(self.filepath).parent.mkdir(parents=True, exist_ok=True)   # create dir if it doesn't exist
        df.to_csv(self.filepath, index=False)



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
        if isinstance(data[0], DefDict):
            return list(map(lambda d: {**dict(d.flatten())}, data))
        return list(map(lambda d: {**dict(d)}, data))

