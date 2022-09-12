from sim.device import SenseBase
from sim.typing import DefDict
import numpy as np
from controller import Node
from typing import Any

class WebotsSense(SenseBase):
    def __init__(self, wb_robot, timestep):
        super().__init__()
        self._robot = wb_robot
        self._timestep = timestep
        self.to_thread = False

    def init(self):
        self._sensors = DefDict(self.sense_space.list_keys())

    def open(self):
        for SENSOR in self.sense_space.keys():
            self._sensors[SENSOR] = self._robot.getDevice(SENSOR)
        self.enable(enable=True)

    def close(self):
        self.enable(enable=False)

    def enable(self, enable, *args, **kwargs):
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
                self.sense_space[key] = sensor.getRangeImage()
            else:
                try:  # webots has two different functions of getValue()
                    self.sense_space[key] = self._filter_nan(sensor.getValue())   # return a scalar value always
                except AttributeError: # then try getValues()
                    self.sense_space[key] = self._filter_nan(sensor.getValues()) # return lists
        return self.sense_space

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

    @staticmethod
    def create_sense_space(sensor_names, *args, **kwargs):
        return DefDict(sensor_names, Any)

