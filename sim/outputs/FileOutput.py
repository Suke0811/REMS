from sim.outputs.OutputBase import OutputBase
from sim.outputs.FileCsvOutput import FileCsvOutput
from pathlib import Path
import pandas as pd
import yaml, json

DEFAULT_EXTENSION = 'yml'

class FileOutput(OutputBase):
    def __init__(self, filepath):
        super().__init__()
        if filepath.find('.') == -1:
            filepath = filepath + '.' + DEFAULT_EXTENSION
        self.filepath = filepath
        self.file_extension = filepath.split('.')[-1]

    def make_output(self):
        """make proper output from the data"""
        if self.file_extension == 'csv':
            csv = FileCsvOutput(self.filepath)
            csv._save_2_file(self._timestamps, self._inpts, self._states, self._outpts, self._info)
        elif self.file_extension == 'yml' or self.file_extension == 'yaml':
            self._to_yaml()
        elif self.file_extension == 'json':
            self._to_json()
        else:
            raise ValueError(f'File extension should be yml, json, or csv, but {self.file_extension} was specified')

    def _to_yaml(self):
        data = self.to_timeseries_dict()
        p = Path(self.filepath).parent.mkdir(parents=True, exist_ok=True)   # create dir if it doesn't exist
        with open(self.filepath, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)

    def _to_json(self):
        data = self.to_timeseries_dict()
        p = Path(self.filepath).parent.mkdir(parents=True, exist_ok=True)   # create dir if it doesn't exist
        with open(self.filepath, "w") as outfile:
            json.dump(data, outfile)
