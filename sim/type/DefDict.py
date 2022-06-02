import numpy as np
from sim.type import DefDictData
from typing import Any


class DefDict:
    def __init__(self, *definition, dtype=Any, rule=None):
        self._definition = DefDictData()
        self._data = DefDictData()
        self.out_rule = rule

        # if args dtype=type was not specified, here we figure out if the last element is type
        if isinstance(definition[-1], type) or definition[-1] is Any:
            dtype=definition[-1]
            definition = definition[0:-1]
        for d in definition:    # add as many definition as you want
            self.add_definition(d, dtype)

    @property
    def data(self):
        return self._data

    @data.getter
    def data(self):
        return self._data

    def data_as(self, definition):
        d = DefDict(definition)
        return d.set_data(self.data)

    def ruled_get(self):
        if self.out_rule is None:
            return self._data
        return self.out_rule.bind(self._data)

    def ruled_set(self, ndata):
        pass

    @data.setter
    def data(self, ndata):
        self.set_data(ndata)

    def set_data(self, ndata):
        if ndata is None:
            return
        if isinstance(ndata, DefDict):
            ndata = ndata.data
        if isinstance(ndata, dict):
            self._dict2dict(ndata)
        elif isinstance(ndata, list):
            self._list2dict(ndata)
        elif isinstance(ndata, np.ndarray):
            self._list2dict(ndata.flatten())
        else:
            self._list2dict([ndata])
        return self

    @data.deleter
    def data(self):
        for k, v in self._definition.items():
            self._data[k] = v()
        self._data = DefDictData(self._data)

    def format_data(self, ndata):
        d = DefDict(self._data, rule=self.out_rule)
        return d.set_data(ndata)


    @property
    def DEF(self):
        return self._definition

    def add_definition(self, ndef, type_=float):
        keys = []
        if isinstance(ndef, dict):
            for k, v in ndef.items():
                if isinstance(v, type):
                    keys.append(k)
                else:
                    self._data[k] = v
                    v = type(v)
            self._definition.update(ndef)
        elif isinstance(ndef, list):
            self._definition.update(dict.fromkeys(ndef, type_))
            keys.extend(ndef)
        elif isinstance(ndef, str):
            self._definition[ndef] = type_
            keys.append(ndef)
        else:
            raise TypeError('You can only add str, dict, or list')
        self.init_data(keys)

    def init_data(self, keys):
        for k, v in self._definition.filter(keys).items():
            try:
                self._data[k] = v()
            except TypeError:
                self._data[k] = None

    def _dict2dict(self, data: dict):
        extra_data = {}
        stored_data_keys = []
        for k, v in data.items():
            if k in self._data.keys():
                self._data[k] = self._enforce_type(self.DEF[k], v)
                stored_data_keys.append(k)
            else:
                extra_data[k] = v
        if not extra_data:
            # TODO: binding implementation
            pass
            self.bind_from(extra_data, stored_data_keys)


    def _list2dict(self, data):
        length = min(len(data), len(self._definition)) - 1
        for i, k in enumerate(self._data):
            if i > length:
                break
            self._data[k] = self._enforce_type(self.DEF[k], data[i])


    def _enforce_type(self, d_type, value):
        if d_type is Any:   # If the type is Any, no enforcement
            ret = value
        try:
            ret = d_type(value)
        except TypeError:
            ret = value
        return ret    # Enforce type in the corresponding definition

    def bind_from(self, data, ignored_keys=None):
        pass
        #def_keys =
        #if ignored_keys is None:


    def assert_data(self, data=None):
        if data is None:
            data = self._data
        else:
            data = DefDictData(data)
        assert (len(data) == len(self.DEF))
        assert (all(type(x) == y for x, y in zip(data.as_list(), self.DEF.as_list())))


if __name__ == '__main__':
    # defined anything I want like below
    store = DefDict(dict(a=float, b=float, c=float))
    print(store.data)
    l = {'a':2.0,'v':4.0}
    a=np.array([2,6])
    store.data = l
    print(store.data.as_list())
    store.assert_data()
    #a.data
    #{'j.0': 0.0, 'j.1': 0.0, 'j.2': 0.0, 'j.3': 0.0, 'j.4': 0.0, 'j.5': 0.0}
    #a.data = [1,2,3,4,5,6]
    #a.data['j.2'] =459459

