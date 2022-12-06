from rems.robots import RobotBase
from concurrent.futures import ThreadPoolExecutor
from rems.sim_handler.ray.DeviceExecutor import DeviceExecutor

class RobotThread(RobotBase):
    def __init__(self, debug_mode=False, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.debug_mode = debug_mode
        self.executor = None

    def add_controller(self, *args, **kwargs):
        pass

    def add_process(self, *args, **kwargs):
        pass

    def add_device(self, device, *args, **kwargs):
        name = device.device_name
        try:
            drive_space = self.drive_space.get(name)
        except KeyError:
            drive_space = None
        try:
            sense_space = self.sense_space.get(name)
        except KeyError:
            sense_space = None
        try:
            device.set_drive_space(drive_space)
        except AttributeError:
            pass
        try:
            device.set_sense_space(sense_space)
        except AttributeError:
            pass
        config = device.config
        for k, c in config.step().items():
            if c == 0.0 or c is None:
                config.step()[k] = self.run.DT
        self.devices.append(DeviceExecutor(device))

    def start_controller(self):
        if self.executor is None:
            self.executor = ThreadPoolExecutor()
        self.cntrl_futs = [self.executor.submit(device.start) for device in self.devices]

    def start_process(self):
        if self.executor is None:
            self.executor = ThreadPoolExecutor()
        self.pro_futs = [self.executor.submit(device.start) for device in self.devices]

    def start_device(self):
        if self.executor is None:
            self.executor = ThreadPoolExecutor()
        self.dev_futs = [self.executor.submit(device.start) for device in self.devices]

    def init(self, *args, **kwargs):
        """Initialization necessary for the robot. call all binded objects' init
        """
        [device.init(self.inpt, self.state, self.outpt) for device in self.devices]



    def reset(self, init_state, t):
        """process necessary to reset the robot without restarting"""
        [device.reset() for device in self.devices]
        #self.futs = [threading.Thread(target=device.start, args=()) for device in self.devices]


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

    def open(self):
        [device.open() for device in self.devices]
        self.enable(True)

    def enable(self, enable):
        [device.enable(enable) for device in self.devices]

    def close(self):
        self.enable(False)
        [device.close() for device in self.devices]






