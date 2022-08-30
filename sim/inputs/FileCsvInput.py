from sim.inputs.FileInput import FileInput
import pandas as pd

SEPARATOR = '.'

class FileCsvInput(FileInput):
    def _open_file(self):
        if self.file_extension == 'csv':
            df = pd.read_csv(self._filepath)
        else:
            raise ValueError(f'File extension should be csv, but {self.file_extension} was specified')
        return df


