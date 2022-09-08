from typing import Any, Union, Tuple
import unyt
from unyt.exceptions import UnitParseError
import numpy as np



Pos = dict(unit='m', dtype=float, drange=(-float('inf'), float('inf')), drange_unit=None, dim=1, default=0.0)
Dist = dict(unit='m', dtype=float, drange=(-float('inf'), float('inf')), drange_unit=None, dim=1, default=0.0)

RESERVED = ['quat', 'rot3d', 'rot2d', 'euler', 'ax_ang', 'ax_ang4d']

MAX=1
MIN=0

class InvalidUnitChangeError(Exception):
    def __init__(self, old_unit, old_dim, new_unit, new_dim):
        self.old_unit = old_unit
        self.old_dim = old_dim
        self.new_unit = new_unit
        self.new_dim = new_dim

    def __str__(self):
        return f"Unit cannot be changed to {self.new_unit} from {self.old_unit} because the dimensions is inconsistent between default {self.old_dim} and new {self.new_dim}"

class UnitType:
    default_unit = 'dimensionless'
    default_dtype = float
    default_dim: Union[int, Tuple[int, int]] = 1
    default_value = None
    default_drange: Tuple[(Union[int, float, str], Union[int, float, str])] = (float('-inf'), float('inf'))
    default_drange_map: Tuple[(Union[int, float, str], Union[int, float, str])] = None
    defualt_drange_scale: Tuple[(Union[int, float], Union[int, float])] = (-1, 1)
    default_conversion_rules: list = []

    def __init__(self, unit=None, drange=None, default=None, dim=None, drange_map=None, drange_scale=None, dtype=None, strict=False, data=None):
        if unit is None:
            unit = self.default_unit
        if drange is None:
            drange = self.default_drange
        if dim is None:
            dim = self.default_dim
        if dtype is None:
            dtype = self.default_dtype
        if drange_map is None:
            drange_map = self.default_drange_map
        if drange_scale is None:
            drange_scale = self.defualt_drange_scale

        self.unit = unit
        self.dtype = dtype
        self.dim = dim
        self.drange = drange
        self.drange_map = drange_map
        self.drange_scale = drange_scale
        self.strict = strict
        self.custom_unit  = None


        if default is None:
            if self.default_value is None:
                default = self.get_default()
            else:
                default = self.default_value
        self.default = default

        # initiate the data with default
        if data is None:
            data = self.default
        self.data = data
        self._rules = self.default_conversion_rules

    @property
    def unit(self):
        return self._unit
    @unit.setter
    def unit(self, val: str):
        if val in RESERVED:
            uval = unyt.unyt_quantity.from_string('dimensionless')
        else:
            try:
                uval = unyt.unyt_quantity.from_string(val)
                default_unit = unyt.unyt_quantity.from_string(self.default_unit)
                if not default_unit.units == unyt.dimensionless:  # if dimensionless, accept all
                    if not default_unit.units.dimensions.simplify() == uval.units.dimensions.simplify():
                        # if the default is not dimensionless, then the new unit should match the default dimensions
                        raise InvalidUnitChangeError(default_unit.units, default_unit.units.dimensions,
                                                     uval.units, uval.units.dimensions)
            except (UnitParseError, ValueError):
                self.custom_unit = val
                uval = unyt.unyt_quantity.from_string('dimensionless')
        self._unit = uval

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
        self._default = self.enforce_type(v)

    @property
    def drange(self):
        return self._drange

    @drange.setter
    def drange(self, val):
        drange = []
        for v in val:
            if v is str:  # is str, put it to unyt
                uv = unyt.unyt_quantity.from_string(v)
                if uv.units == unyt.dimensionless:  # in case dimensionless
                    uv *= self.unit
            else:  # otherwise, assume the same unit
                uv = self.enforce_unit(v)
            drange.append(uv)
        self._drange = tuple(drange)

    @property
    def drange_map(self):
        return self._drange_map

    @drange_map.setter
    def drange_map(self, val):
        if val is None:
            self._drange_map = None
            return
        drange = []
        for v in val:
            if callable(v) or None:
                drange.append(v)
                continue
            if isinstance(v, str):  # is str, put it to unyt
                uv = unyt.unyt_quantity.from_string(v)
                if uv.units == unyt.dimensionless:  # in case dimensionless
                    uv *= self.unit
            else:  # otherwise, assume the same unit
                uv = self.enforce_unit(v)
            drange.append(uv)
        self._drange_map = tuple(drange)

    @property
    def dtype(self):
        return self._dtype

    @dtype.setter
    def dtype(self, v):
        self._dtype = v

    @property
    def drange_scale(self):
        return self._drange_scale

    @drange_scale.setter
    def drange_scale(self, v):
        self._drange_scale = v

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, val):
        self._data = self.to(val)     # store date in mutable


    ######################

    def get_default(self):
        if self._is_defdict(self.dtype):
            default = self.dtype.dict()
        elif self.dtype is np.array:
            default = np.zeros(self.dim)
        else:
            try:
                default = self.dtype()
            except (TypeError, AttributeError):
                default = 0.0
        return default

    def enforce_unit(self, val):
        return self._to_unyt(val, self.unit)  # unyt

    def _to_unyt(self, val, unit):
        if val is None:
            return val
        if isinstance(val, str):
            val = unyt.unyt_quantity.from_string(val)
        if not isinstance(unit, str):
            if isinstance(val, unyt.unyt_array):
                val = val.to(unit)
            else:
                val = val * unit
        return val  # unyt

    def enforce(self, val, vdef=None):
        vdef = self._create_class_instance(vdef)
        if vdef.unit.units == self.unit.units:
            return self.enforce_type(val)
        elif self.unit.units == unyt.percent:
            val = vdef.to_percent(val, vdef=self)
        elif self.unit.units == unyt.count:
            val = self.to_count(val, vdef=self)
        elif vdef.unit.units == unyt.count:
            val = vdef.from_count(val)
        elif vdef.unit.units == unyt.percent:
            val = self.from_percent(val, vdef=vdef)

        # ruled = self._ruled_conversion(vdef.unit, val, self.unit)
        # if ruled is not None:
        #     return ruled    # if ruled is not None, then return ruled value

        elif not isinstance(val, unyt.unyt_array):
            val = vdef.enforce_unit(val)  # unyt
            val = val.to(self.unit)
        else:
            val = val.to(self.unit)
        return self.enforce_type(val)  # unyt to proper type

    def to(self, val, vdef=None):
        vdef = self._create_class_instance(vdef)
        if vdef.unit.units == self.unit.units:
            return self.enforce_type(val)

        elif vdef.unit.units == unyt.percent:
            val = self.to_percent(val, vdef=vdef)

        elif vdef.unit.units == unyt.count:
            val = vdef.to_count(val, vdef)
        elif self.unit.units == unyt.count:
            val = self.from_count(val)

        elif self.unit.units == unyt.percent:
            val = vdef.from_percent(val, vdef=self)

        # ruled = self._ruled_conversion(vdef.unit, val, self.unit)
        # if ruled is not None:
        #     return ruled    # if ruled is not None, then return ruled value

        elif not isinstance(val, unyt.unyt_array):
            val = self.enforce_unit(val)    # unyt
            val = val.to(vdef.unit)
        else:
            val = val.to(vdef.unit)
        return vdef.enforce_type(val) # unyt to proper type

    def _create_class_instance(self, vdef):
        if vdef is None:
            vdef = self
        elif not isinstance(vdef, UnitType):
            vdef = vdef()
        return vdef

    def from_count(self, val):
        """
        drange_unit should be like
        = (val_0, val_1, ...,  val_n)
        or
        = (val_0, func, val_1, val_2, ..., val_n)
        if you skip func, then linear interpolation will be used
        :param val:
        :return:
        """
        scaled_val = self.enforce_unit(val)
        if self.drange_map is None:
            return scaled_val
        drange_map = iter(self.drange_map)
        drange = iter(self.drange)
        sv_map = next(drange_map)
        sv = next(drange)

        while True:
            try:
                nv_map = next(drange_map)
                lv = next(drange)
                if sv <= val and val <= lv:
                    if not callable(nv_map):  # if not callable not function -> this is the next value
                        lv_map = nv_map
                        func = lambda o_val, sv, lv, sv_map, lv_map: ( ((lv_map - sv_map) / (lv - sv)) * (o_val - sv) + sv_map)
                    else:
                        func = nv_map
                        lv_map = next(drange_map)  # if callable, then we need to get next value

                    scaled_val = func(scaled_val, sv, lv, sv_map, lv_map)
                    break
                else:
                    lv_map = nv_map
            except StopIteration:
                break
            sv_map = lv_map
            sv = lv
        return scaled_val

    def to_count(self, val, vdef=None):
        """
        drange_unit should be like
        = (val_0, val_1, ...,  val_n)
        or
        = (val_0, func, val_1, val_2, ..., val_n)
        if you skip func, then linear interpolation will be used
        :param val:
        :return:
        """
        if vdef is None:
            vdef = self
        scaled_val = vdef._to_unyt(val, vdef.drange_map[0].units)
        if self.drange_map is None:
            return scaled_val
        drange_map = iter(self.drange_map)
        drange = iter(self.drange)
        sv_map = next(drange_map)
        sv = next(drange)

        while True:
            try:
                nv_map = next(drange_map)
                lv = next(drange)
                if sv <= val and val <= lv:
                    if not callable(nv_map):  # if not callable not function -> this is the next value
                        lv_map = nv_map
                        func = lambda o_val, sv, lv, sv_map, lv_map: (((lv - sv)/(lv_map - sv_map)) * (o_val - sv_map) + sv)
                    else:
                        func = nv_map
                        lv_map = next(drange_map)  # if callable, then we need to get next value

                    scaled_val = func(scaled_val, sv, lv, sv_map, lv_map)
                    break
                else:
                    lv_map = nv_map
            except StopIteration:
                break
            sv_map = lv_map
            sv = lv
        return scaled_val


    def to_percent(self, val, vdef=None):
        if vdef is None:
            vdef = self
        if all([vd == float('inf') or vd == -float('inf') for vd in self.drange]):
            raise ValueError(f'{val} cannot converted to % because data range is {self.drange}')

        if not isinstance(val, unyt.unyt_array):
            val = self.enforce_unit(val)
        p_val = (val - self.drange[MIN]) / (self.drange[MAX] - self.drange[MIN]) # percent 0-1 scale
        s_val = p_val * (vdef.drange_scale[MAX] - vdef.drange_scale[MIN]) + vdef.drange_scale[MIN] # scale back to as defined
        return vdef.enforce_type(100*s_val) # percent)

    def from_percent(self, val, vdef=None):
        if vdef is None:
            vdef = self
        if all([vd == float('inf') or vd == -float('inf') for vd in self.drange]):
            raise ValueError(f'{val} cannot converted from % from a value because data range is {self.drange}')

        val = val/100
        #scale percent i.e. [-100%, 100%], [0,100%]
        p_val = (val - vdef.drange_scale[MIN]) / (vdef.drange_scale[MAX] - vdef.drange_scale[MIN])
        s_val = p_val * (self.drange[MAX] - self.drange[MIN]) + self.drange[MIN]
        return self.enforce_unit(s_val)


    def enforce_type(self, val):
        """
        This will enforce the type and strip off the unit if unyt variables are given
        """
        enforced_val = val
        if self._is_defdict(self.dtype):
            self.dtype.set(val)
            return self.dtype

        if self.dtype is np.array:  # special handling for np.array
            if isinstance(val, unyt.unyt_array):
                enforced_val = val.to_ndarray()
            elif not isinstance(val, self.dtype):
                enforced_val = np.array(val)
        else:
            if isinstance(self.dtype, type) and not isinstance(val, self.dtype): # otherwise apply the type
                try:
                    enforced_val = self.dtype(val)
                except (TypeError, AttributeError): # some type like Any throw the error
                    pass
        return np.clip(enforced_val, * tuple([d.value for d in self.drange]))

    def _ruled_conversion(self, from_unit, val, to_unit):
        if self._rules:
            for rule in self._rules:
                if rule.if_rule_apply(from_unit) and rule.if_rule_out_contains(to_unit):
                    return rule.bind(dict(from_unit=val))
                if rule.if_rule_apply(to_unit) and rule.if_rule_out_contains(from_unit):
                    return rule.inv_bind(to_unit)
        return None

    def _is_defdict(self, data):
        try:
            if data.is_DefDict():
                return True
        except AttributeError:
            pass
        return False



    def __str__(self):
        return f"{self.unit.units, tuple([d.to_string() for d in self.drange]), self.dtype}"

    def __repr__(self):
        return  f"{self.unit.units, tuple([d.to_string() for d in self.drange]), self.dtype}"
