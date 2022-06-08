import numpy as np
from sim.typing import DefDict
SEPARATOR = '.'


class DefDictData(dict):
    def __init__(self, *args, **kwargs ):
        super().__init__(*args, **kwargs)
        self.suffixes = []
        self.attrs = []

    def ndarray(self, reshape: tuple=None):
        data_list = self.list()
        return np.array(data_list).reshape(reshape)

    def ndsquare(self):
        s = np.sqrt(len(self))
        if np.ceil(s) != s:
            raise ValueError('Cannot make the data size {} to square matrix'.format(len(self)))
        return self.ndarray((int(s), int(s)))

    def ndtall(self):
        return self.ndarray((len(self), 1))

    def list(self):
        return list(self.values())

    def list_keys(self):
        ret = []
        for k, v in self.items():
            ret.append(k)
        return list(self.keys())

    def filter(self, keys):
        if isinstance(keys, dict):
            keys = list(keys.keys())
        if isinstance(keys, str):
            return self.get(keys)
        if isinstance(keys, int):
            keys = [keys]
        if not isinstance(keys, list):
            raise TypeError('keys must be either string, list, or dict')
        d = DefDictData({k: self.get(k) for k in keys})
        for s in self.suffixes:
            d._add_suffix(s)
        return d

    def filter_data(self, data):
        if isinstance(data, dict):
            data = list(data.values())
        if not isinstance(data, list):
            data = [data]

        found = []
        for i, value in enumerate(self.list()):
            for search_for in data:
                if value == search_for:
                    found.append(i)
        keys = list(self.keys())
        vals = list(self.values())

        return DefDictData({keys[index]: vals[index] for index in found})

    def remove_prefix(self, prefix=None):
        d = DefDictData()
        for k in self.list_keys():
            k_seq = k.split(SEPARATOR)
            if len(k_seq) <= 1:
                break
            if prefix is None:
                k_seq.pop(0)
                d.setdefault(SEPARATOR.join(k_seq), self.get(k))
            else:
                if prefix in k_seq:
                    k_seq.remove(prefix)
                    d.setdefault(SEPARATOR.join(k_seq), self.get(k))
        for s in self.suffixes:
            d._add_suffix(s)
        return d

    def _add_attr(self, name):
        if name in self.attrs:
            raise AttributeError('the prefix is already registered')
        self.attrs.append(name)
        def method(self, ids=None):
            if ids is None:
                return self.remove_prefix(name)
            elif not isinstance(ids, list):
                ids = [ids]
            return self.remove_prefix(name).filter(list(map(str, ids)))

        setattr(self, name, method.__get__(self))

    def _add_suffix(self, name):
        if name in self.suffixes:
            raise AttributeError('the suffix is already registered')
        self.suffixes.append(name)
        def method(self):
            data = DefDictData()
            for k, v in self.items():
                if isinstance(v, DefDict) or isinstance(v, dict):
                    data[k] = v.get(name)
            return data

        setattr(self, name, method.__get__(self))


if __name__ == '__main__':
    d = DefDictData()
    d['j.0'] = {'pos': 43, 'vel': 100}
    d['j.2'] = {'pos': 43, 'vel': 100}
    d['j.1'] = {'pos': 43, 'vel': 100}
    d._add_attr('j')
    d._add_suffix('pos')
    print(d.j([1]).pos())
    print(d.pos)
    pass
