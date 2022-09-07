import copy
import logging
import numpy as np
from typing import Any
from sim.typing.UnitType import UnitType
SEPARATOR = '.'


class DefDict:
    reserved = ['get', 'set', 'keys', 'list', 'list_keys', 'ndtall']
    def __init__(self, definition=None, dtype=Any, name=None, prefixes=None, suffixes=None, format_rule=None, shape=None, rules=None, nested_def=True):
        self._definition = dict()
        self._data = dict()
        self._name = name
        self.format_rule = format_rule
        self.shape = shape
        self.rules = []
        self.suffixes = []
        self.prefixes = []
        if definition is not None:
            self.add_def(definition, dtype=dtype, prefixes=prefixes, suffixes=suffixes, format_rule=format_rule,
                         shape=shape, rules=rules, nested_def=nested_def)

    def add_def(self, definition, dtype=Any, prefixes=None, suffixes=None, format_rule=None, shape=None, rules=None, nested_def=True):
        if format_rule is not None:
            self.format_rule = format_rule
        if shape is not None:
            self.shape = shape
        if not isinstance(rules, list) and rules is not None:
            rules = [rules]
        if rules is not None:
            self.rules.extend(rules)

        if isinstance(suffixes, dict):
            suffixes = list(suffixes.keys())
        elif not isinstance(suffixes, list):
            suffixes = [suffixes]

        if isinstance(definition, tuple):
            for d in definition:  # add as many definition as you want
                suffixes.extend(self._set_defs(d, dtype, nested_def))
                if isinstance(d, DefDict) and suffixes:
                    self._add_suffix(d.list_keys())
        else:
            suffixes.extend(self._set_defs(definition, dtype, nested_def))

        if prefixes is not None:
            if isinstance(prefixes, dict):
                prefixes = list(prefixes.keys())
            self._add_prefixes(prefixes)
        if suffixes is not None:
            self._add_suffixes(suffixes)
        return self


    def ndarray(self, reshape: tuple = None):
        data_list = self.list()
        if reshape is None:
            reshape = self.shape
        return np.array(data_list).reshape(reshape)

    def ndtall(self):
        return self.ndarray((len(self._data), 1))

    def list(self):
        return list(self.values())

    def list_keys(self):
        return list(self._data.keys())

    def get(self, key=None):
        if key is None:
            ret = self.list()[0]    # get the first element when no key has been specified
        else:
            ret = self._data[key][0]
        return ret

    def dict(self):
        return self.data    # return dictionary

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
        for k in self.keys():
            k_seq = k.split(SEPARATOR)
            if len(k_seq) <= 1:
                continue
            if prefix is None:
                k_seq.pop(0)
                d._set_defs({SEPARATOR.join(k_seq): self.DEF[k]})
                d._data[SEPARATOR.join(k_seq)] = self._data.get(k)
            else:
                if prefix in k_seq:
                    k_seq.remove(prefix)
                    d._set_defs({SEPARATOR.join(k_seq): self.DEF[k]})
                    d._data[SEPARATOR.join(k_seq)] = self._data.get(k)
        return d

    def _add_prefixes(self, prefixes):
        if not isinstance(prefixes, list):
            prefixes = [prefixes]
        for name in prefixes:
            self._add_prefix(name)

    def _add_prefix(self, name):
        if name in self.reserved:
            raise AttributeError(f'the prefix {name} is reserved for DefDict')
        if name in self.prefixes:
            logging.debug(f'the prefix {name} is already registered')
            return
        self.prefixes.append(name)

        def method(self, ids=None):
            if ids is None:
                return self.remove_prefix(name)
            elif not isinstance(ids, list):
                ids = [ids]
            ids = list(map(str, ids))
            return self.remove_prefix(name).filter(ids)
        setattr(self, name, method.__get__(self))

    def _add_suffixes(self, suffixes):
        if not isinstance(suffixes, list):
            suffixes = [suffixes]
        for name in suffixes:
            if name is None:
                continue
            self._add_suffix(name)

    def _add_suffix(self, name):
        if name in self.suffixes:
            logging.debug('the suffix is already registered')
            return
        self.suffixes.append(name)
        def method(self):
            d = copy.deepcopy(self)
            d.clear()
            for k, v in self.items():
                if isinstance(v, DefDict):
                    d._set_defs({k: v.DEF[name]})
                    d._data[k] = v._data.get(name)
                elif isinstance(v, dict):
                    d._set_defs({k: v[name]})
                    d._data[k] = v.get(name)
                elif k == name:
                    d._set_defs({k: self.DEF[k]})
                    d._data[k] = v
            return d
        setattr(self, name, method.__get__(self))

    @property
    def data(self):
        ret = {}
        for k, v in self._data.items():
            if not k.startswith('_'):  # protected keys are not accessible normally
                ret[k] = v[0]
        return ret

    @data.setter
    def data(self, ndata):
        self.set(ndata)

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, val):
        self._name = val


    def set(self, ndata):
        if ndata is None:
            return self
        if self.rules is not None:
            ret = self._apply_rules(ndata)
            if ret is not None:
                ndata = ret
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
            ndata = list(ndata)
            self._list2dict(ndata)
        elif isinstance(ndata, np.ndarray):
            self._list2dict(ndata)
        else:
            self._list2dict([ndata])
        return self

    def _apply_rules(self, data):
        ret = None
        if isinstance(data, dict) or isinstance(data, DefDict):
            for rule in self.rules:
                if rule.bind_from is None:
                    continue
                keys = rule.bind_from.list_keys()
                if all(elem in list(data.keys()) for elem in keys):
                    ret = rule.bind(data)
                    if ret is not None:
                        break
        return ret

    def set_rule(self, rule):
        if self.rules is None:
            self.rules = []
        self.rules.append(rule)
        return self

    def clear_rules(self):
        self.rules = None
        return self

    def set_name(self, name):
        self._name = name
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

    def _set_defs(self, ndef, dtype=Any, nested_dict=False):
        keys = []
        suffixes = []
        if isinstance(ndef, dict):
            for k, v in ndef.items():
                if isinstance(v, dict) and nested_dict:
                    v = DefDict(v)
                    suffixes.extend(v.list_keys())
                if isinstance(v, DefDict):
                    self._definition[k] = v
                   # keys.append(k)
                    self._data[k] = [v]
                elif isinstance(v, type) or v is Any:
                    self._definition[k] = dtype
                    keys.append(k)
                    self._data[k] = 0.0
                else:
                    self._definition[k] = type(v)
                    self._data[k] = [v]

        elif isinstance(ndef, list):
            self._definition.update(dict.fromkeys(ndef, dtype))
            keys.extend(ndef)
        elif isinstance(ndef, str):
            self._definition[ndef] = dtype
            keys.append(ndef)
        else:
            raise TypeError('You can only add str, dict, or list')
        self.init_data(keys)
        return suffixes

    def init_data(self, keys):
        for k, v in self._definition.items():
            if k in keys:
                if v is Any:
                    self._data[k] = [float()] # what should be the init value?
                elif isinstance(v, type):
                    self._data[k] = [v()]
                else:
                    self._data[k] = [v]   # maybe a different way of initialization?

    def _from_defdict(self, data):
        for k, v in data.items():
            if k in self._data.keys():
                if isinstance(self.DEF[k], DefDict):
                    self._data[k][0].set(v)
                else:
                    self._data[k][0] = self._enforce_type(self.DEF[k], v, data.DEF[k])

    def _dict2dict(self, data: dict):
        for k, v in data.items():
            if k in self._data.keys():
                if self.DEF[k] is DefDict:
                    self._data[k][0].set(v)
                else:
                    self._data[k][0] = self._enforce_type(self.DEF[k], v)


    def _list2dict(self, data):
        length = min(len(data), len(self._definition)) - 1
        for i, key in enumerate(self._data.keys()):
            if i > length:
                break
            if key.startswith('_'):
                continue
            if isinstance(self.DEF[key], DefDict):
                self._data[key][0].set(data[i])
            else:
                self._data[key][0] = self._enforce_type(self.DEF[key], data[i])

    def _check_if_iterable(self, item):
        try:
            iter(item)
            return True
        except TypeError:
            return False

    def _enforce_type(self, d_type, value, vdef=None):
        if isinstance(d_type, UnitType):
            ret = self._unit_type(d_type, value, vdef)
        if isinstance(d_type, DefDict):
            ret = d_type.set(value)
        elif d_type is Any:   # If the type is Any, no enforcement
            ret = value
        else:
            try:
                ret = d_type(value)
            except (TypeError, AttributeError):
                ret = value
            if not isinstance(ret, d_type):
                raise TypeError(f'{ret} is not type of {d_type}')
        return ret    # Enforce type in the corresponding definition

    def _unit_type(self, dtype, value, vdef=None):
        return dtype.to(value, vdef)

    def bind(self, bind_rule, val=None):
        if val is None:
            val = self.dict()
        self.set(bind_rule.bind(val))
        return self

    def inv_bind(self, bind_rule, val=None):
        if val is None:
            val = self.dict()
        self.set(bind_rule.inv_bind(val))
        return self

    def assert_data(self, data=None):
        if data is None:
            data = self._data
        else:
            data = dict(data)
        assert (len(data) == len(self.DEF))
        assert (all(type(x) == y for x, y in zip(data.values(), self.DEF.values())))

    def _to_key_list(self, keys):
        if isinstance(keys, dict):
            keys = list(keys.keys())
        if isinstance(keys, str):
            keys = [keys]
        if isinstance(keys, int):
            keys = [str(keys)]
        if not isinstance(keys, list):
            raise TypeError('keys must be either string, int, list, or dict')
        return keys

    def filter(self, keys): #ToDo support all iteratibe
        key_list = []
        if isinstance(keys, tuple):
            for t in keys:
                key_list.extend(self._to_key_list(t))
        else:
            key_list.extend(self._to_key_list(keys))

        key_list = map(str, key_list)
        d = copy.deepcopy(self)
        d.clear()
        for k in key_list:
            if k in self.keys():
                d._set_defs({k: self.DEF[k]})
                d._data[k] = self._data.get(k)
        return d    #DefDict

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
        d = copy.deepcopy(self)
        d.clear()
        for index in found:
            d._set_defs({keys[index]: vals[index]})
            d._data[keys[index]] = vals[index]
        return d

    def flat_list(self):
        ret = []
        for elem in self.values():
            try:
                i = iter(elem)
                for e in elem:
                    ret.append(e)
            except TypeError:
                ret.append(elem)
        return ret

    def to_float(self):
        for k, v in self.items():
            if isinstance(v, np.ndarray):
                v.astype(float)
            elif isinstance(v, DefDict):
                v.to_float()
            else:
                try:
                    self._data[k][0] = float(v)
                except ValueError:
                    pass
        return self

    def to_int(self):
        for k, v in self.items():
            if isinstance(v, np.ndarray):
                v.astype(int)
            elif isinstance(v, DefDict):
                v.to_int()
            else:
                try:
                    self._data[k][0] = int(v)
                except ValueError:
                    pass
        return self


###################################################################
    #dictionary methods
    def clear(self):  # real signature unknown; restored from __doc__
        """ D.clear() -> None.  Remove all items from D. """
        self._data.clear()
        self.DEF.clear()

    def copy(self):  # real signature unknown; restored from __doc__
        """ D.copy() -> a shallow copy of D """
        return copy.copy(self)

    def clone(self):
        """
        Deep copy of itself
        """
        return copy.deepcopy(self)

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
        return self.set(ndata)

    def update(self, ndata):  # known special case of dict.update
        """
        same as set
        """
        return self.set(ndata)

    def values(self):  # real signature unknown; restored from __doc__
        """ D.values() -> an object providing a view on D's values """
        return self.data.values()

    def __setitem__(self, key, value):
        if key in self.DEF.keys():
            if isinstance(self._data[key][0], DefDict):  # for nested def dict
                self._data[key][0].set(value)
            else:
                self._data[key][0] = value
        else:
            raise KeyError(f'Key {key} is not in definition')

    def __getitem__(self, item):
        return self._data.__getitem__(item)[0]


##############################################################
    # magic methods
    def __len__(self):
        return len(d._data)

    def __repr__(self):
        return {k: v[0].__str__() for k, v in self._data.items()}.__str__()

    def __str__(self):
        return {k: v[0].__str__() for k, v in self._data.items()}.__str__()

##############################################################
    # Math operations
    def _math(self, other, func, immutable=True):
        if immutable:
            current = self.clone()
        else:
            current = self
        if np.isscalar(other):  # if scalar, then add the value to all elements
            for k in current._data.keys():
                current._data[k][0] = func(current._data[k][0], other)
        else:  # other wise element wise
            if not isinstance(other, DefDict):
                # if not DefDict, create one assuming
                other_defdict = self.clone()
                other_defdict.init_data(other_defdict.list_keys())
                other_defdict.set(other)
            else:
                other_defdict = other
            # sum for corresponding keys
            for k, o in zip(current.filter(other_defdict.keys()).list_keys(),
                        other_defdict.filter(other_defdict.list_keys()).list()):
                current._data[k][0] = func(current._data[k][0], o)
        return current

    def __iter__(self):
        return iter(self.values())

    def __add__(self, other):
        return self._math(other, lambda v, o: v + o, immutable=True)

    def __iadd__(self, other):
        return self._math(other, lambda v, o: v + o, immutable=False)

    def __sub__(self, other):
        return self._math(other, lambda v, o: v - o, immutable=True)

    def __isub__(self, other):
        return self._math(other, lambda v, o: v - o, immutable=False)

    def __mul__(self, other):
        return self._math(other, lambda v, o: v * o, immutable=True)

    def __imul__(self, other):
        return self._math(other, lambda v, o: v * o, immutable=False)

    def __floordiv__(self, other):
        return self._math(other, lambda v, o: v // o, immutable=True)

    def __ifloordiv__(self, other):
        return self._math(other, lambda v, o: v // o, immutable=False)

    def __truediv__(self, other):
        return self._math(other, lambda v, o: v / o, immutable=True)

    def __itruediv__(self, other):
        return self._math(other, lambda v, o: v / o, immutable=False)

    def __mod__(self, other):
        return self._math(other, lambda v, o: v % o, immutable=True)

    def __imod__(self, other):
        return self._math(other, lambda v, o: v % o, immutable=True)

    def __pow__(self, other, modulo=None):  #Todo: modulo implementation
        return self._math(other, lambda v, o: v ** o, immutable=True)

    def __ipow__(self, other, modulo=None):  #Todo: modulo implementation
        return self._math(other, lambda v, o: v ** o, immutable=False)

#############################
    def _compare(self, other, func):
        ret = []
        current = self
        if np.isscalar(other):  # if scalar, then compare the value to all elements
            for k in current._data.keys():
                ret.append(func(current._data[k][0], other))
        else:  # other wise element wise
            if not isinstance(other, DefDict):
                # if not DefDict, create one assuming
                other_defdict = self.clone()
                other_defdict.init_data(other_defdict.list_keys())
                other_defdict.set(other)
            else:
                other_defdict = other
            # sum for corresponding keys
            for k, o in zip(current.filter(other_defdict.keys()).list_keys(),
                        other_defdict.filter(other_defdict.list_keys()).list()):
                ret.append(func(current._data[k][0], o))
        return ret

    def __lt__(self, other):
        return self._compare(other, lambda v, o: v < o)

    def __le__(self, other):
        return self._compare(other, lambda v, o: v <= o)

    def __eq__(self, other):
        return self._compare(other, lambda v, o: v == o)

    def __ne__(self, other):
        return self._compare(other, lambda v, o: v != o)

    def __ge__(self, other):
        return self._compare(other, lambda v, o: v >= o)

    def __gt__(self, other):
        return self._compare(other, lambda v, o: v > o)

    def __round__(self, n=None):
        d = self.clone()
        for k, v in d.items():
            d._data[k][0] = round(v, n)
        return d

    @property
    def name(self):
        return self._name



if __name__ == '__main__':
    d =DefDict({'leg.0':DefDict({'j.0':1, 'j.1':2}, prefixes=['j']),'leg.1':DefDict({'j.0':1, 'j.1':2},prefixes=['j'])},prefixes=['leg'], suffixes=['j'])
    print(d)
