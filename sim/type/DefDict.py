import numpy as np
from sim.type.DefDictData import DefDictData
from typing import Any

class DefDict:
    def __init__(self, definition, type_=Any):
        if isinstance(definition, dict):
            self._definition = DefDictData(definition)
        elif isinstance(definition, list):
            self._definition = DefDictData.fromkeys(definition, type_)
        elif isinstance(definition, str):
            self._definition = DefDictData({definition: type_})
        else:
            raise TypeError('definition must be dict, list or str')

        self._data = DefDictData()
        for k, v in self._definition.items():
            try:
                self._data[k] = v()
            except TypeError:
                self._data[k] = None
        self._data = DefDictData(self._data)

    @property
    def data(self):
        return self._data

    @data.getter
    def data(self):
        return self._data

    def data_as(self, definition):
        d = DefDict(definition)
        return d.set_data(self.data)

    @data.setter
    def data(self, ndata):
        self.set_data(ndata)

    def set_data(self, ndata):
        if ndata is None:
            return
        if isinstance(ndata, dict):
            self._dict2dict(ndata)
        elif isinstance(ndata, list):
            self._list2dict(ndata)
        elif isinstance(ndata, np.ndarray):
            self._list2dict(ndata)
        else:
            self._list2dict([ndata])
        return self._data

    @data.deleter
    def data(self):
        for k, v in self._definition.items():
            self._data[k] = v()
        self._data = DefDictData(self._data)

    @property
    def DEF(self):
        return self._definition

    def add_definition(self, ndef, type_=float()):
        if isinstance(ndef, dict):
            self._definition.update(ndef)
        elif isinstance(ndef, list):
            self._definition.update(dict.fromkeys(ndef, type_))
        elif isinstance(ndef, str):
            self._definition[ndef] = type_
        else:
            raise TypeError('You can only add str, dict, or list')
        # TODO: modify to initiate data with added def

    def _dict2dict(self, data: dict):
        for k, v in data.items():
            if self._data.get(k) is not None:
                self._data[k] = self._enforce_type(self.DEF[k], v)

    def _list2dict(self, data):
        length = min(len(data), len(self._definition)) - 1
        for i, k in enumerate(self._data):
            if i > length:
                break
            self._data[k] = self._enforce_type(self.DEF[k],data[i])


    def _enforce_type(self, d_type, value):
        if d_type is Any:   # If the type is Any, no enforcement
            return value
        return d_type(value)    # Enforce type in the corresponding definition

    def assert_data(self, data=None):
        if data is None:
            data = self._data
        else:
            data = DefDictData(data)
        assert (len(data) == len(self.DEF))
        assert (all(type(x) == y for x, y in zip(data.as_list(), self.DEF.as_list())))





if __name__ == '__main__':
    store = DefDict(dict(a=float,b=float,c=float))
    print(store.data)
    l = {'a':2.0,'v':4.0}
    a=np.array([2,6])
    store.data = a
    print(store.data.as_list())
    store.assert_data()

