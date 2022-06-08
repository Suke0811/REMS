import numpy as np
from typing import Any
from sim.typing.DefDictData import DefDictData



class DefDict:
    def __init__(self, *definition, dtype=Any, rule=None, name=None, prefixes=None, inherit=True):
        self._definition = DefDictData()
        self._data = DefDictData()
        self._name = name
        self.out_rule = rule
        self.attrs = prefixes

        # if args dtype=type was not specified, here we figure out if the last element is type
        if isinstance(definition[-1], type) or definition[-1] is Any:
            dtype=definition[-1]
            definition = definition[0:-1]
        for d in definition:    # add as many definition as you want
            self.add_definition(d, dtype)
            if isinstance(d, DefDict) and inherit:
                pass



        if prefixes is not None:
            self.add_prefix(prefixes)

    def get(self, format=None, flatten=None):
        if format is None:
            return self._data
        else:
            return self._data.filter(format)

    def list(self, format=None):
        return self.get(format).list()

    def keys(self, to_int=False):
        if to_int:
            rets = []
            for k in self._data.list_keys():
                rets.append(int(k))
            return rets
        else:
            return self._data.list_keys()

    def ndarray(self, reshape=None, format=None):
        return self.get(format).ndarray(reshape)

    def ndsquare(self, format=None):
        return self.get(format).ndsquare()

    def ndtall(self, format=None):
        return self.get(format).ndtall()

    def as_ruled(self):
        if self.out_rule is None:
            return self._data
        return self.out_rule.bind(self._data)

    def add_prefix(self, prefixes):
        if not isinstance(prefixes, list):
            prefixes = [prefixes]
        for p in prefixes:
            self._add_attr(str(p))

    def _add_attr(self, name):
        self._data._add_attr(name)
        def method(self, ids=None):
            return getattr(self._data, name)(ids)
        setattr(self, name, method.__get__(self))

    @property
    def data(self):
        return self._data

    @data.getter
    def data(self):
        return self.get()

    @data.setter
    def data(self, ndata):
        self.set(ndata)

    def set(self, ndata):
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

    def format(self, ndata):
        d = DefDict(self._data, rule=self.out_rule)
        return d.set(ndata)

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

    def assert_data(self, data=None):
        if data is None:
            data = self._data
        else:
            data = DefDictData(data)
        assert (len(data) == len(self.DEF))
        assert (all(type(x) == y for x, y in zip(data.list(), self.DEF.list())))

    def __str__(self):
        return self._data.__str__()

