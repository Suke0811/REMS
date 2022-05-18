import numpy as np


class DefDictData(dict):
    def as_numpy(self, reshape: tuple=None):
        data_list = self.as_list()
        return np.array(data_list).reshape(reshape)

    def as_square_numpy(self):
        s = np.sqrt(len(self))
        if np.ceil(s) != s:
            raise ValueError('Cannot make the data size {} to square matrix'.format(len(self)))
        return self.as_numpy((int(s), int(s)))

    def as_tall_numpy(self):
        return self.as_numpy((len(self),1))

    def as_list(self):
        ret = []
        for k, v in self.items():
            ret.append(v)
        return ret

    def key_as_list(self):
        ret = []
        for k, v in self.items():
            ret.append(k)
        return ret

    def filter(self, keys):
        if isinstance(keys, dict):
            keys = list(keys.keys())
        if isinstance(keys, str):
            return self.get(keys)
        if not isinstance(keys, list):
            raise TypeError('keys must be either string, list, or dict')

        return DefDictData({k: self.get(k) for k in keys})

    def filter_data(self, data):
        if isinstance(data, dict):
            data = list(data.values())
        if not isinstance(data, list):
            data = [data]

        found = []
        for i, value in enumerate(self.as_list()):
            for search_for in data:
                if value == search_for:
                    found.append(i)
        keys = list(self.keys())
        vals = list(self.values())

        return DefDictData({keys[index]: vals[index] for index in found})

    def get_key_suffix(self):
        rets = []
        keys = self.key_as_list()
        for k in keys:
            rets.append(k.split('.')[-1])
        return rets

    def get_prefix(self):
        rets = []
        keys = self.key_as_list()
        for k in keys:
            rets.append(k.split('.')[0])
        return rets
