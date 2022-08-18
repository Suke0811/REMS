from sim.robots.RunConfig import RunConfig
from sim.device.BasicDeviceBase import BasicDeviceBase
from sim.robots.RobotDefBase import RobotDefBase
from sim.sim_handler.ray.RayWrapper import RayWrapper



class RobotBase(RobotDefBase, BasicDeviceBase):
    def __init__(self, *args, **kwargs):
        """init with a specific initial stat) """
        super().__init__(*args, **kwargs)
        self._t_minus_1 = 0.0       # delta t may not be a constant
        self.info = {}
        # Devices
        self.devices_data = []
        self.devices = []
        self.drivers = []
        self.sensors = []
        self.observers = []
        # Run settings
        self.run = RunConfig()
        self.home_position = self.joint_space

    def add_device(self, device, *args, **kwargs):
        self.devices_data.append((device, args, kwargs))


    def init(self, *args, **kwargs):
        """Initialization necessary for the robot. call all binded objects' init
        """
        for data in self.devices_data:
            d, args, kwargs = data
            device = d(*args, **kwargs)
            if device.to_thread:
                device = RayWrapper(device, name=device.device_name, cache=False)
            self.devices.append(device)
            config = device.config
            if config.get('drive').get('on'):
                self.drivers.append(device)
            if config.get('sense').get('on'):
                self.sensors.append(device)
            if config.get('observe_state').get('on'):
                self.observers.append(device)
            device.init()

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
        for device in self.drivers:
            device.drive(self.joint_space.list(), timestamp, block=False)

    def sense(self):
        """generate the sensor reading
        :return output"""
        for device in self.sensors:
            self.outpt.data = device.sense(block=False)
        return self.outpt

    def observe_state(self):
        """get current state"""
        for device in self.observers:
            self.state.data = device.observe_state(block=False)
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
