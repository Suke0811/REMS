import numpy as np


class UnitFloat(float):
    def __new__(self, value, unit):
        return float.__new__(self, value)
    def __init__(self, val, unit):
        float.__init__(val)
        self.unit = unit

pass



