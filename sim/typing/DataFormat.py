from dataclasses import dataclass, field
from typing import Any, Union, Callable
import unyt
import numpy as np

class DataFormatBase:
    """
    Base calss for DefDict Data element.
    Data field is private and shouldn't be overwritten
    """
    _data: Any = None


class DataFormat(DataFormatBase):
    """
    DataFormat for
    """
    dtype: type = float
    unit: str = None
    drange: tuple[(Any, Any)] = (-float('inf'), float('inf'))
    drange_unit: Union[tuple[(str, str)], tuple[(str, Callable, str)]] = None
    dim: Union[int, tuple] = None
    default: Any = None

    def __init__(self, unit=unit, drange=drange, drange_unit=drange_unit, dim=dim, default=default, dtype=dtype):
        self.unit = unit
        self.drange = drange
        self.drange_unit = drange_unit
        self.dim = dim
        self.default = default
        self.dtype = dtype

    @property
    def unit(self):
        return unyt.unyt_quantity.from_string(self._unit)

    @unit.setter
    def unit(self, v):
        self._unit = unyt.unyt_quantity.from_string(v)

    @property
    def dim(self):
        return self._dim
    @dim.setter
    def dim(self, v):
        if v is None:
            self._dim = np.ndim(self._data)
        else:
            self._dim = v

    @property
    def default(self):
        if self._default is None:
            try:
                self._default = self.dtype()
            except (TypeError, AttributeError):
                pass
        return self._default

    @default.setter
    def default(self, v):
        try:
            self._default = self.dtype(v)
        except (TypeError, AttributeError):
            self._default = v

    @property
    def drange_unit(self):
        if self.drange_unit is None:
            self._drange_unit = (self.unit, self.unit)
        return self._drange_unit
    @drange_unit.setter
    def drange_unit(self, v):
        self._drange_unit = v
    @property
    def drange(self):
        return self._drange

    @drange.setter
    def drange(self, v):
        self._drange = v

    def to_data(self, val, vdef=None):
        if vdef is None or not isinstance(vdef, DataFormat) or vdef.unit is None:
            nval = val
        else:
            nval = val*vdef.unit.to(self.unit)
        return nval

    def scale_value(self, val):
        """
        drange_unit should be like
        = (val_0, val_1, ...,  val_n)
        or
        = (val_0, func, val_1, val_2, ..., val_n)
        if you skip func, then linear interpolation will be used
        :param val:
        :return:
        """
        val = val * self.unit
        scaled_val = val
        i = iter(self.drange_unit)
        sv = next(i)
        while True:
            try:
                nv = next(i)
                if not callable(nv):  # if not callable not function -> this is the next value
                    lv = nv
                    func = lambda val, sv, lv: (val - sv) / (lv - sv)
                else:
                    func = nv
                    lv = next(i)    # if callable, then we need to get next value

                if sv <= val and val <= lv:
                    scaled_val = func(val, sv, lv)
                    break
            except StopIteration:
                break
        return scaled_val


    def to_percent(self, val, vdef):
        if all([vd==float('inf') or vd==-float('inf') for vd in vdef.drange]):
            raise ValueError(f'{val} cannot converted to % because data range is {vdef.drange}')


    def to_unit(self):
        pass

    def to_count(self):
        pass

    def enforce_type(self, val, vdef):
        return





class Pos(DataFormat):
    unit: Any = 'm'     # unit for the data to be stored
    dtype: type = float # python type of the value, float, int, etc.
    drange: tuple[(Any, Any)] = (-float('inf'), float('inf'))
                        # data range. used to limit the value in range and to convert the variable to unitless
    drange_unit = None  # if the data range units are different from data unit
    dim: Union[int, tuple] = 1
                        # dimensionality of the data
    default: Any = None # Default value



class MetaWithFooClassProperty(type):
    @property
    def foo(cls):
        """The foo property is a function of the class -
        in this case, the trivial case of the identity function.
        """
        return cls

class FooClassProperty(metaclass=MetaWithFooClassProperty):
    @property
    def foo(self):
        """access the class's property"""
        return self.foo

    @foo.setter
    def foo(self, v):
        print('instance')
        self.foo = v
pass



