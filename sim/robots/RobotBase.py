import time

from sim.robots.RunConfig import RunConfig
from sim.device.BasicDeviceBase import BasicDeviceBase
from sim.robots.RobotDefBase import RobotDefBase
from sim.typing import DefDict
from sim.sim_handler.ray.DeviceExecutor import DeviceExecutor
import inspect
from concurrent.futures import ThreadPoolExecutor

import threading


class RobotBase(RobotDefBase, BasicDeviceBase):
    DEVICE_LIST = []

    def __init__(self, *args, **kwargs):
        """init with a specific initial stat) """
        super().__init__(*args, **kwargs)
        self._t_minus_1 = 0.0       # delta t may not be a constant
        self._t_real = None
        self.info = {}
        # Devices
        self.devices = []
        # Run settings
        self.run = RunConfig()
        self.home_position = self.joint_space

    def add_device(self, device, *args, **kwargs):
        # curframe = inspect.currentframe()
        # calframe = inspect.getouterframes(curframe, 2)
        # if  calframe[1][3] != 'init_devices':
        #     raise SyntaxError(f'add_device should be called only in init_device, called from {calframe[1][3]}')
       # if isinstance()
        name = device.device_name
        try:
            drive_space = self.drive_space.get(name)
        except KeyError:
            drive_space = None
        try:
            sense_space = self.sense_space.get(name)
        except KeyError:
            sense_space = None
        try: device.set_drive_space(drive_space)
        except AttributeError: pass
        try: device.set_sense_space(sense_space)
        except AttributeError: pass
        config = device.config
        for k, c in config.step().items():
            if c == 0.0 or c is None:
                config.step()[k] = self.run.DT
        self.devices.append(DeviceExecutor(device))

    def init_devices(self):
        pass


    def init(self, *args, **kwargs):
        """Initialization necessary for the robot. call all binded objects' init
        """
        [device.init(self.inpt, self.state, self.outpt) for device in self.devices]

    def reset(self, init_state, t):
        """process necessary to reset the robot without restarting"""
        self.futs = [threading.Thread(target=device.start, args=()) for device in self.devices]
        [t.start() for t in self.futs]

    def control(self, inpt, timestamp):
        self.inpt.set(inpt)
        self.joint_space.set(self.inpt)
        return self.joint_space

    def drive(self, inpt, timestamp):
        """drive the robot to the next state
        :param inpts: left, right wheel velocities
        """
        self.inpt.update(inpt)
        for device in self.devices:
            device.drive(self.inpt, timestamp)

    def sense(self):
        """generate the sensor reading
        :return output"""
        for device in self.devices:
            ret = device.sense()
            if ret is not None:
                self.outpt.update(ret)
        return self.outpt

    def observe_state(self):
        """get current state"""
        for device in self.devices:
            ret = device.observe_state()
            if ret is not None:
                self.state.update(ret)
        return self.state

    def clock(self, t):
        if self.run.realtime:
            if self._t_real is None:
                self._t_real = time.perf_counter()
                t += self.run.DT
            else:
                t += time.perf_counter() - self._t_real
                self._t_real = time.perf_counter()
        else:
            self._t_minus_1 = t
            t += self.run.DT
        return t

    def open(self):
        [device.open() for device in self.devices]
        self.enable(True)

    def enable(self, enable):
        [device.enable(enable) for device in self.devices]

    def close(self):
        self.enable(False)
        [device.close() for device in self.devices]

    @classmethod
    def drive_space_def(cls, *args, **kwargs):
        ret = dict()
        for device in cls.DEVICE_LIST:
            try:
                space = device.create_drive_space(*args, **kwargs)
                ret.setdefault(device.device_name, space)
            except AttributeError:
                pass
        return DefDict(ret)

    @classmethod
    def sense_space_def(cls, *args, **kwargs):
        ret = dict()
        for device in cls.DEVICE_LIST:
            try:
                space = device.create_sense_space(*args, **kwargs)
                ret.setdefault(device.device_name, space)
            except AttributeError:
                pass
        return DefDict(ret)
