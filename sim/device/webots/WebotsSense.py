from sim.device import SenseBase
from sim.typing import DefDict
import numpy as np
from controller import Node

class WebotsSense(SenseBase):
    def __init__(self, wb_robot, timestep, sensor_definition: DefDict):
        super().__init__()
        self._sensors = DefDict(sensor_definition.list_keys())
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
                if sensor.getNodeType == Node.LIDAR:
                    sensor.enablePointCloud()
            else:
                sensor.disable()

    def sense(self):
        for key, sensor in self._sensors.items():
            if sensor.getNodeType() == Node.LIDAR:
                self.outpt[key] = sensor.getRangeImage()
            else:
                try:  # webots has two different functions of getValue()
                    self.outpt[key] = self._filter_nan(sensor.getValue())   # return a scalar value always
                except AttributeError: # then try getValues()
                    self.outpt[key] = self._filter_nan(sensor.getValues()) # return lists
        return self.outpt

    @staticmethod
    def _filter_nan(values):
        if values is None:
            return 0.0
        if not isinstance(values, list):
            return values

        ret = []
        for v in values:
            if np.isnan(v):
                v = 0.0
            ret.append(v)
        return ret  # this case return list

