from sim.robots.RunConfig import RunConfig
from sim.device.BasicDeviceBase import BasicDeviceBase
from sim.robots.RobotDefBase import RobotDefBase
from sim.sim_handler.ray.RayWrapper import RayWrapper
from sim.sim_handler.ray.DeviceExecutor import DeviceExecutor

from concurrent.futures import ThreadPoolExecutor



class RobotBase(RobotDefBase, BasicDeviceBase):
    def __init__(self, *args, **kwargs):
        """init with a specific initial stat) """
        super().__init__(*args, **kwargs)
        self._t_minus_1 = 0.0       # delta t may not be a constant
        self.info = {}
        # Devices
        self.devices = []
        # Run settings
        self.run = RunConfig()
        self.home_position = self.joint_space

    def add_device(self, device, *args, **kwargs):
        self.devices.append(DeviceExecutor(device))


    def init(self, *args, **kwargs):
        """Initialization necessary for the robot. call all binded objects' init
        """
        [device.init(self.inpt, self.state, self.outpt) for device in self.devices]
        self.executor = ThreadPoolExecutor()
        self.futs = [self.executor.submit(device.start) for device in self.devices]

    def reset(self, init_state, t):
        """process necessary to reset the robot without restarting"""
        pass

    def control(self, inpt, timestamp):
        self.inpt.set(inpt)
        self.joint_space.set(self.inpt)
        return self.joint_space

    def drive(self, inpt, timestamp):
        """drive the robot to the next state
        :param inpts: left, right wheel velocities
        """
        self.joint_space.set(self.control(inpt, timestamp))
        for device in self.devices:
            device.drive(self.joint_space, timestamp)

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
        self._t_minus_1 = t
        return t + self.run.DT

    def open(self):
        [device.open() for device in self.devices]
        self.enable(True)

    def enable(self, enable):
        [device.enable(enable) for device in self.devices]

    def close(self):
        self.enable(False)
        [device.close() for device in self.devices]
        self.executor.shutdown()
