import numpy as np

SEPARATOR = '.'


class DefDictData(dict):
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

        return DefDictData({k: self.get(k) for k in keys})

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

    def get_key_suffix(self):
        rets = []
        keys = self.list_keys()
        for k in keys:
            rets.append(k.split('.')[-1])
        return rets

    def get_prefix(self):
        rets = []
        keys = self.list_keys()
        for k in keys:
            rets.append(k.split('.')[0])
        return rets

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
        return d


    def _add_attr(self, name):
        def method(self, ids=None):
            if ids is None:
                return self.remove_prefix(name)
            else:
                return self.remove_prefix(name).filter(ids)

        setattr(self, name, method.__get__(self))




if __name__ == '__main__':
    d = DefDictData()
    d['j.0'] = 1
    d['j.2'] = 2
    d['j.1'] = 4
    d._add_attr('j')
    print(d.j())
    pass
