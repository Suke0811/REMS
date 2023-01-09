import numpy as np
from rems.typing import DefDict, UnitType, MapRule
from typing import Union
from scipy.spatial.transform import Rotation as R


"""
Here are predefined physical quantities that are predefined. You can define your own by extending UnitType.
You can also change the settings when you create a instance as follow:
dict(x=Pos(unit='cm', drange=('-1km', '1000m'), dtype=int), y=Pos())
Note that if you do:
Pos_cm = Pos(unit='cm', drange=('-1km', '1000m'), dtype=int)
dict(x=Pos_cm, y=Pos_cm)
changing values in Pos_cm will affect the both
"""


class Example(UnitType):
    # unit
    default_unit = 'dimensionless'
    # default value for this unit, if none then we do dtype(), i.e. float() = 0.0
    default_value = None
    # data type
    default_dtype = float
    # dimension i.e. 1D array of 3 columns -> 3 or 2D array 2 by 3 -> (2, 3)
    default_dim = 1
    # data range for this unit
    default_drange: tuple = (float('-inf'), float('inf'))
    # data mapping to let you create your own unit conversion
    # unit='count'; drange=(0, 4095); drange_map=('0.01m', '1m')
    # This indicates 0 count corresponds to 0.01m and 4096 to 1m and inbetween is linear interpolation
    # drange_map=('-1m', func, '1m') and takes func(val, sv, lv). this let you define custom interpolation
    # you can also have more points. If no func is specified, then it'll be linear interpolation
    # drange=(0, 1024, 2048, 3072 ,4096); drange_map=('10mm', '0.1m', '0.5m', func, '0.75m', ,'1m')
    default_drange_map: tuple = None
    # data scale. (-1, 1) -> -100% to 100%. (0, 1) -> 0% to 100%
    defualt_drange_scale: tuple = (-1, 1)


class Time(UnitType):
    default_unit = 's'
    default_drange = (0, float('inf'))
    defualt_drange_scale = (0, 1)

class Pos(UnitType):
    default_unit = 'm'

class Vel(UnitType):
    default_unit = 'm/s'

class Acc(UnitType):
    default_unit = 'm/s**2'

class Ang(UnitType):
    default_unit = 'rad'

class AngVel(UnitType):
    default_unit = 'rad/s'

class AngAcc(UnitType):
    default_unit = 'rad/s**2'

class Mass(UnitType):
    default_unit = 'kg'
    default_drange = (0, float('inf'))
    defualt_drange_scale = (0, 1)

class Percent(UnitType):
    default_unit = 'percent'
    defualt_drange_scale = (0, 1)

class Count(UnitType):
    default_unit = 'count'
    default_dtype = int
    default_drange = (0, 4095)
    defualt_drange_scale = (0, 1)

class CountSensor(UnitType):
    default_unit = 'count'
    default_dtype = int
    default_drange = (0, 4095)
    default_drange_map = ('0.1m', '1m')
    defualt_drange_scale = (0, 1)

class VelCreate(UnitType):
    default_unit = 'rad/s'
    default_drange = (-5,5)

class CountVel(UnitType):
    default_unit = 'count'
    default_dtype = int
    default_drange = (-500, 500)
    default_drange_map = ('-5rad/s', '5rad/s')
    defualt_drange_scale = (0, 1)

class Float(UnitType):
    defualt_unit = 'dimensionless'
    default_dtype = float

class Int(UnitType):
    defualt_unit = 'dimensionless'
    default_dtype = int
