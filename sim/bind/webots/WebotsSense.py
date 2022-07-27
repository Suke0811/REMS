from sim.bind import SenseBase
from sim.typing import DefDict
import numpy as np

class WebotsSense(SenseBase):
    def __init__(self, wb_robot, timestep, sensor_definition: DefDict):
        super().__init__()
        self._sensors = sensor_definition.clone()
        self.outpt = sensor_definition.clone()
        self._robot = wb_robot
        self._timestep = timestep

    def init(self):
        self.open()
        self.enable(enable=True)

    def open(self):
        for SENSOR in self.outpt.keys():
            self._sensors[SENSOR] = self._robot.getDevice(SENSOR)

    def close(self):
        self.enable(enable=False)

    def enable(self, enable):
        for sensor in self._sensors:
            if enable:
                sensor.enable(self._timestep)
            else:
                sensor.disable()

    def sense(self):
        sensor_value = []
        for sensor in self._sensors:
            try: # webots has two different functions of getValue()
                sensor_value.append(self._filter_nan(sensor.getValue()))
            except AttributeError: # then try getValues()
                sensor_value.append(self._filter_nan(sensor.getValues()))
        self.outpt.set(sensor_value)
        return self.outpt

    @staticmethod
    def _filter_nan(values):
        if not isinstance(values, list):
            return values
        if values is None:
            return 0.0
        for v in values:
            if not np.isnan(v):
                return v
        return None

