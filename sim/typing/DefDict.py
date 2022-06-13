import copy

import numpy as np
from typing import Any

SEPARATOR = '.'


class DefDict:
    reserved = ['get', 'set', 'keys', 'list', 'list_keys', 'ndtall']
    def __init__(self, definition, dtype=Any, name=None, prefixes=None, suffixes=None, format_rule=None, ):
        self._definition = dict()
        self._data = dict()
        self._name = name
        self.format_rule = format_rule
        self.suffixes = []
        self.prefixes = []

        if isinstance(definition, tuple):
            for d in definition:    # add as many definition as you want
                self.add_definition(d, dtype)
                if isinstance(d, DefDict) and suffixes:
                    self._add_suffix(d.list_keys())
        else:
            self.add_definition(definition, dtype)

        if prefixes is not None:
            if isinstance(prefixes, dict):
                prefixes = list(prefixes.keys())
            self._add_prefix(prefixes)
        if suffixes is not None:
            if isinstance(suffixes, dict):
                suffixes = list(suffixes.keys())
            self._add_suffixes(suffixes)

    def ndarray(self, reshape: tuple = None):
        data_list = self.list()
        return np.array(data_list).reshape(reshape)

    def ndsquare(self):
        s = np.sqrt(len(self._data))
        if np.ceil(s) != s:
            raise ValueError('Cannot make the data size {} to square matrix'.format(len(self._data)))
        return self.ndarray((int(s), int(s)))

    def ndtall(self):
        return self.ndarray((len(self._data), 1))

    def list(self):
        return list(self.values())

    def list_keys(self):
        return list(self._data.keys())

    def get(self, format=None, flatten=None):
        if format is None:
            return self.data
        else:
            return self.filter(format)

    def keys(self, to_int=False):
        if to_int:
            return map(int, self.list_keys())
        else:
            return self.list_keys()

    def as_ruled(self):
        if self.format_rule is None:
            return self.data
        return self.format_rule.bind(self.data)

    def remove_prefix(self, prefix=None):
        d = copy.deepcopy(self)
        d.clear()
        for k in self.list_keys():
            k_seq = k.split(SEPARATOR)
            if len(k_seq) <= 1:
                break
            if prefix is None:
                k_seq.pop(0)
                d.add_definition({SEPARATOR.join(k_seq): self.DEF[k]})
                d._data[SEPARATOR.join(k_seq)] = self._data.get(k)
            else:
                if prefix in k_seq:
                    k_seq.remove(prefix)
                    d.add_definition({SEPARATOR.join(k_seq): self.DEF[k]})
                    d._data[SEPARATOR.join(k_seq)] = self._data.get(k)
        return d

    def _add_prefix(self, prefixes):
        if not isinstance(prefixes, list):
            prefixes = [prefixes]
        for name in prefixes:
            if name in self.reserved:
                raise AttributeError(f'the prefix {name} is reserved for DefDict')
            if name in self.prefixes:
                raise AttributeError(f'the prefix {name} is already registered')
            self.prefixes.append(name)

            def method(self, ids=None):
                if ids is None:
                    return self.remove_prefix(name)
                elif not isinstance(ids, list):
                    ids = [ids]
                return self.remove_prefix(name).filter(list(map(str, ids)))

            setattr(self, name, method.__get__(self))

    def _add_suffixes(self, suffixes):
        if not isinstance(suffixes, list):
            suffixes = [suffixes]
        for name in suffixes:
            self._add_suffix(name)


    def _add_suffix(self, name):
        if name in self.suffixes:
            raise AttributeError('the suffix is already registered')
        self.suffixes.append(name)
        def method(self):
            d = copy.deepcopy(self)
            d.clear()
            for k, v in self.items():
                if isinstance(v, DefDict):
                    d.add_definition({k: v.DEF[name]})
                    d._data[k] = v._data.get(name)
                elif isinstance(v, dict):
                    d.add_definition({k: v[name]})
                    d._data[k] = v.get(name)
                elif k == name:
                    d.add_definition({k: self.DEF[k]})
                    d._data[k] = v
            return d
        setattr(self, name, method.__get__(self))

    @property
    def data(self):
        return {k: v[0] for k, v in self._data.items()}

    @data.setter
    def data(self, ndata):
        self.set(ndata)

    def set(self, ndata):
        if ndata is None:
            return
        if self.format_rule is not None:
            try:
                ndata = self.format_rule.inv_bind(ndata)
            except:
                pass
        if isinstance(ndata, DefDict):
            ndata = ndata.data
        if isinstance(ndata, dict):
            self._dict2dict(ndata)
        elif isinstance(ndata, list):
            self._list2dict(ndata)
        elif isinstance(ndata, tuple):
            k,v = ndata
            ndata = {k:v}
            self._dict2dict(ndata)
        elif isinstance(ndata, np.ndarray):
            self._list2dict(ndata.flatten())
        else:
            self._list2dict([ndata])
        return self

    def format(self, ndata):
        """format dose not change the stored data, but format the input into current definition"""
        d = copy.deepcopy(self)
        return d.set(ndata)

    @property
    def DEF(self):
        return self._definition

    def get_DEF(self, keys):
        key_lists = []
        if isinstance(keys, DefDict):
            key_lists = keys.list_keys()
        elif isinstance(keys, dict):
            key_lists = list(keys.keys())
        elif isinstance(keys, tuple):
            k, v = keys
            key_lists = [k]

        DEFs = []
        for k in key_lists:
            if k in self.DEF.keys():
                DEFs.append(self.DEF[k])
            else:
                raise (f'{k} is not in definition')
        return DEFs

    def add_definition(self, ndef, type_=float):
        keys = []
        if isinstance(ndef, dict):
            for k, v in ndef.items():
                if isinstance(v, type):
                    keys.append(k)
                else:
                    self._data[k] = [v]

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
        for k, v in self._definition.items():
            if k in keys:
                try:
                    self._data[k] = [v()]
                except TypeError:
                    self._data[k] = [v]   # maybe a different way of initialization?

    def _dict2dict(self, data: dict):
        extra_data = {}
        stored_data_keys = []
        for k, v in data.items():
            if k in self._data.keys():
                self._data[k][0] = self._enforce_type(self.DEF[k], v)
                stored_data_keys.append(k)
            else:
                extra_data[k][0] = v
        if extra_data:
            # TODO: binding implementation
            return
            self.bind_from(extra_data, stored_data_keys)

    def _list2dict(self, data):
        length = min(len(data), len(self._definition)) - 1
        for i, key in enumerate(self._data):
            if i > length:
                break
            if isinstance(key, tuple):
                k, v = key
                self._data[k][0] = self._enforce_type(self.DEF[k], v)
            else:
                self._data[key][0] = self._enforce_type(self.DEF[key], data[i])

    def _enforce_type(self, d_type, value):
        if d_type is Any:   # If the type is Any, no enforcement
            ret = value
        else:
            try:
                ret = d_type(value)
            except TypeError:
                ret = value
        return ret    # Enforce type in the corresponding definition

    def bind(self, bind_rule):
        self.set(bind_rule.bind(self.get()))

    def assert_data(self, data=None):
        if data is None:
            data = self._data
        else:
            data = dict(data)
        assert (len(data) == len(self.DEF))
        assert (all(type(x) == y for x, y in zip(data.values(), self.DEF.values())))

    def filter(self, keys):
        if isinstance(keys, dict):
            keys = list(keys.keys())
        if isinstance(keys, str):
            return self._data[keys][0]
        if isinstance(keys, int):
            keys = [str(keys)]
        if not isinstance(keys, list):
            raise TypeError('keys must be either string, int, list, or dict')

        keys = map(str, keys)
        d = copy.deepcopy(self)
        d.clear()
        d.add_definition({k: self.DEF[k] for k in keys})
        d.set({k: self._data[k][0] for k in keys})
        return d    #DefDict

    def filter_data(self, data):
        raise NotImplemented
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
        return DefDict({keys[index]: vals[index] for index in found})

    def __str__(self):
        return self.data.__str__()

###################################################################
    #dictionary methods
    def clear(self):  # real signature unknown; restored from __doc__
        """ D.clear() -> None.  Remove all items from D. """
        self._data.clear()
        self.DEF.clear()

    def copy(self):  # real signature unknown; restored from __doc__
        """ D.copy() -> a shallow copy of D """
        return copy.copy(self)

    def items(self):  # real signature unknown; restored from __doc__
        """ D.items() -> a set-like object providing a view on D's items """
        return self.data.items()

    def pop(self, keys, d=None):  # real signature unknown; restored from __doc__
        """
        D.pop(k[,d]) -> v, remove specified key and return the corresponding value.

        If key is not found, default is returned if given, otherwise KeyError is raised
        """
        rets = []
        if not isinstance(keys, list):
            k = [keys]
        for k in keys:
            if k in self._data.keys():
                rets.append(self._data.get(k)[0])
                self.init_data(k)
            else:
                rets.append(d)
        return rets

    def popitem(self, *args, **kwargs):  # real signature unknown
        """
        Remove and return a (key, value) pair as a 2-tuple.

        Pairs are returned in LIFO (last-in, first-out) order.
        Raises KeyError if the dict is empty.
        """
        keys = [k for k, v in args]
        self.pop(keys)

    def setdefault(self, ndata):  # real signature unknown
        """
        Insert key with a value of default if key is not in the dictionary.
        Return the value for key if key is in the dictionary, else default.
        """
        self.set(ndata)

    def update(self, ndata):  # known special case of dict.update
        """
        same as set
        """
        self.set(ndata)

    def values(self):  # real signature unknown; restored from __doc__
        """ D.values() -> an object providing a view on D's values """
        return self.data.values()

    def __setitem__(self, key, value):
        if key in self.DEF.keys():
            self._data.__setitem__(key, [value])
        else:
            raise KeyError(f'Key {key} is not in definition')

    def __getitem__(self, item):
        return self._data.__getitem__(item)[0]



