from dataclasses import dataclass, field
from typing import Any, Union, Callable, Tuple
import unyt
import numpy as np


Pos = dict(unit='m', dtype=float, drange=(-float('inf'), float('inf')), drange_unit=None, dim=1, default=0.0)
Dist = dict(unit='m', dtype=float, drange=(-float('inf'), float('inf')), drange_unit=None, dim=1, default=0.0)

MIN = 0
MAX = 1

@dataclass
class DataType:
    unit: str = 'dimensionless'
    dtype: type = float
    drange: tuple[(Any, Any)] = field(default_factory=lambda: (-float('inf'), float('inf')))
    drange_map: tuple = None
    dim = 1
    default = None
    scale: tuple[(Any, Any)] = field(default_factory=lambda: (-1, 1))

    def __post_init__(self):
        self._sys_unit = unyt.unyt_quantity.from_string(self.unit)
        drange = []
        for v in self.drange:
            if v is str:    # is str, put it to unyt
                uv = unyt.unyt_quantity.from_string(v)
                if uv.units is unyt.dimensionless:  # in case dimensionless
                    uv *= self._sys_unit

            else:   # otherwise, assume the same unit
                uv = v * self._sys_unit
            drange.append(uv)
        self._sys_drange = tuple(drange)
        self._data = self.get_default()

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, val):
        self.data = val

######################3

    def get_default(self):
        if self.default is None:
            if self.dtype is np.array:
                default = np.zeros(self.dim)
            else:
                try:
                    default = self.dtype()
                except (TypeError, AttributeError):
                    default = 0.0
        return default

    def to_unit(self, val, vdef=None):
        if vdef is None or not issubclass(vdef, DataType):
            nval = val
        else:
            nval = val*vdef._sys_unit.to(self.unit)
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
        val = val * self._sys_unit
        scaled_val = val
        i = iter(self.drange_map)
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

    def to_percent(self, val, vdef=None):
        s_val = None
        if all([vd==float('inf') or vd==-float('inf') for vd in vdef.drange]):
            raise ValueError(f'{val} cannot converted to % because data range is {vdef.drange}')
        else:
#            if
            u_val = val*vdef._sys_unit  # add unit
            p_val = (u_val - self._sys_drange[MIN]) / (self._sys_drange[MAX] - self._sys_drange[MIN])
            s_val = p_val * (self.scale[MAX] - self.scale[MIN]) - self.scale[MIN]
        return s_val


    def to_unit(self, val, v_def=None):
        if v_def is None:
            v_def = self
        unit_val = (val * v_def._sys_unit).to(self._sys_unit)
        return self.enforce_type(unit_val)


    def to_count(self):
        pass

    def enforce_type(self, val):
        #enforced_val =
        if self.dtype is np.array:
            enforced_val = val
        return









@dataclass
class DataFormatter:
    """
    DataFormat for
    """
    def __init__(self, unit: str = 'dimensionless', dtype: type = float,
                 drange: tuple[(Any, Any)] = None, drange_unit: tuple = None, dim=None, default=None):
        self._dtype = dtype
        self._unit = unit
        if drange is None:
            drange = (-float('inf'), float('inf'))
        self._drange = drange
        self._drange_unit = drange_unit
        if dim is None:
            try:
                dim = np.dim(self.dtype())
            except (TypeError, AttributeError):
                dim = 1
        self._dim = dim
        if default is None:
            try:
                default = self.dtype()
            except (TypeError, AttributeError):
                default = float()
        self._default = default




    def to_data(self, val, vdef=None):
        if vdef is None or not issubclass(vdef, DataFormatter):
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

    def to_unit(self, val, v_def=None):
        if v_def is None:
            v_def = self
        unit_val = (val * v_def.unit).to(self.unit)
        return self.enforce_type(unit_val)


    def to_count(self):
        pass

    def enforce_type(self, val):
#        enforced_val =
        if self.dtype is np.array:
            enforced_val = val
        return


class UnitType:
    default_unit = 'm'
    default_dtype = float
    default_drange: tuple[(Union[int, float, str], Union[int, float, str])] = ('-1m', '1m')
    default_drange_map: tuple = None
    default_dim = 1
    default_value = None
    defualt_drange_scale: tuple[(Union[int, float], Union[int, float])] = (-1, 1)

    def __init__(self, unit=default_unit, drange=None, default=default_value, dim=None, drange_map=None, drange_scale=None, dtype=default_dtype, ):
        if drange is None:
            drange = self.default_drange
        if dim is None:
            dim = self.default_dim
        if drange_map is None:
            drange_map = self.default_drange_map
        if drange_scale is None:
            drange_scale = self.defualt_drange_scale


    @property
    def unit(self):
        return self._unit
    @unit.setter
    def unit(self, val:str):
        self._unit = unyt.unyt_quantity.from_string(val)

    @property
    def dim(self):
        return self._dim
    @dim.setter
    def dim(self, v):
        self._dim = v

    @property
    def default(self):
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

    @property
    def dtype(self):
        return self._dtype

    @dtype.setter
    def dtype(self, v):
        self._dtype = v


    def to_unit(self, val, v_def=None):
        if v_def is None:
            v_def = self
        unit_val = (val * v_def._sys_unit).to(self._sys_unit)
        return self.enforce_type(unit_val)

    unit: str = 'dimensionless'
    dtype: type = float
    drange: tuple[(Any, Any)] = field(default_factory=lambda: (-float('inf'), float('inf')))


    def __post_init__(self):
        self._sys_unit = unyt.unyt_quantity.from_string(self.unit)
        drange = []
        for v in self.drange:
            if v is str:  # is str, put it to unyt
                uv = unyt.unyt_quantity.from_string(v)
                if uv.units is unyt.dimensionless:  # in case dimensionless
                    uv *= self._sys_unit

            else:  # otherwise, assume the same unit
                uv = v * self._sys_unit
            drange.append(uv)
        self._sys_drange = tuple(drange)
        self._data = self.get_default()

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, val):
        self.data = val

    ######################3

    def get_default(self):
        if self.default is None:
            if self.dtype is np.array:
                default = np.zeros(self.dim)
            else:
                try:
                    default = self.dtype()
                except (TypeError, AttributeError):
                    default = 0.0
        return default

    def to_unit(self, val, vdef=None):
        if vdef is None or not issubclass(vdef, DataType):
            nval = val
        else:
            nval = val * vdef._sys_unit.to(self.unit)
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
        val = val * self._sys_unit
        scaled_val = val
        i = iter(self.drange_map)
        sv = next(i)
        while True:
            try:
                nv = next(i)
                if not callable(nv):  # if not callable not function -> this is the next value
                    lv = nv
                    func = lambda val, sv, lv: (val - sv) / (lv - sv)
                else:
                    func = nv
                    lv = next(i)  # if callable, then we need to get next value

                if sv <= val and val <= lv:
                    scaled_val = func(val, sv, lv)
                    break
            except StopIteration:
                break
        return scaled_val

    def to_percent(self, val, vdef=None):
        s_val = None
        if all([vd == float('inf') or vd == -float('inf') for vd in vdef.drange]):
            raise ValueError(f'{val} cannot converted to % because data range is {vdef.drange}')
        else:
            #            if
            u_val = val * vdef._sys_unit  # add unit
            p_val = (u_val - self._sys_drange[MIN]) / (self._sys_drange[MAX] - self._sys_drange[MIN])
            s_val = p_val * (self.scale[MAX] - self.scale[MIN]) - self.scale[MIN]
        return s_val

    def to_unit(self, val, v_def=None):
        if v_def is None:
            v_def = self
        unit_val = (val * v_def._sys_unit).to(self._sys_unit)
        return self.enforce_type(unit_val)

    def to_count(self):
        pass

    def enforce_type(self, val):
        # enforced_val =
        if self.dtype is np.array:
            enforced_val = val
        return

pass


