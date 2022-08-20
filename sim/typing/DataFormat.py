from dataclasses import dataclass, field
from typing import Any, Union
import unyt
import numpy as np

@dataclass
class DataFormatBase:
    """
    Base calss for DefDict Data element.
    Data field is private and shouldn't be overwritten
    """
    __data: Any = None



@dataclass
class DataFormat(DataFormatBase):
    """
    DataFormat for
    """
    dtype: type = None
    unit: Any = None
    range: tuple[(Any, Any)] = field(default_factory=lambda: (-float('inf'), float('inf')))
    drange_unit: tuple[(Any, Any)] = None
    dim: Union[int, tuple] = None
    default: Any = None
    default_unit: Any = None
    coord: Any = None



@dataclass
class Pos(DataFormat):
    unit: Any = 'm'     # unit for the data to be stored
    dtype: type = float # python type of the value, float, int, etc.
    drange: list = field(default_factory=lambda: (-float('inf'), float('inf')))
                        # data range. used to convert the variable to unitless
    drange_unit = None  # if the data range units are different from data unit
    dim: Union[int, tuple] = 1
                        # dimensionality of the data
    default: Any = None # Default value

pass



