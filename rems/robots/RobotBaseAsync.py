import time

from rems.robots.RunConfig import RunConfig
from rems.device.BasicDeviceBase import BasicDeviceBase
from rems.robots.RobotDefBase import RobotDefBase
from defdict import DefDict
from rems.sim_handler.thread import DeviceExecutor
from concurrent.futures import ThreadPoolExecutor


class RobotBaseAsync(RobotDefBase, BasicDeviceBase):
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
        self.executor = None
        self._processes = []
        self._pro_futs = []

    def add_device(self, device, *args, **kwargs):
        pass


    def init_devices(self, *args, **kwargs):
        pass

    def add_process(self,  process):
        self._processes.append(process)

    def init_process(self):
        [pro.init() for pro in self._processes]

    def reset_process(self):
        if self.executor is None:
            self.executor = ThreadPoolExecutor()
        self._pro_futs = [self.executor.submit(pro.start) for pro in self._processes]


    def init(self, *args, **kwargs):
        """Initialization necessary for the robot. call all binded objects' init
        """
        [device.init(self.inpt, self.state, self.outpt) for device in self.devices]


    def reset(self, init_state, t):
        """process necessary to reset the robot without restarting"""
        if self.executor is None:
            self.executor = ThreadPoolExecutor()
        self.futs = [self.executor.submit(device.start) for device in self.devices]


    def control(self, inpt, timestamp):
        self.inpt.set(inpt)
        self.joint_space.set(self.inpt)
        return self.joint_space

    def drive(self, inpt, timestamp):
        """drive the robot to the next state
        :param inpts: left, right wheel velocities
        """
        for device in self.devices:
            device.drive(inpt, timestamp)

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
