from rems.robots import RobotBase
from concurrent.futures import ThreadPoolExecutor
from rems.sim_handler.thread import DeviceExecutor, ProcessExecutor

class RobotThread(RobotBase):
    def __init__(self, debug_mode=False, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.debug_mode = debug_mode
        self.executor = None
        self._controllers = []
        self._processes = []
        self._devices = []

    def add_process(self, process, *args, **kwargs):
        self._processes.append(ProcessExecutor(process))

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
        self._devices.append(DeviceExecutor(device))

    def init_devices(self):
        self.start_device()



    def start_process(self):
        if self.executor is None:
            self.executor = ThreadPoolExecutor()
        self.pro_futs = [self.executor.submit(process.start) for process in self._processes]
        def exception_call(fut):
            fut.result()
        [fut.add_done_callback(exception_call) for fut in self.pro_futs]

    def start_device(self):
        if self.executor is None:
            self.executor = ThreadPoolExecutor()
        self.dev_futs = [self.executor.submit(device.start) for device in self._devices]

    def init(self, *args, **kwargs):
        """Initialization necessary for the robot. call all binded objects' init
        """
        self._processes = [ProcessExecutor(p) for p in self.pros]
        [device.init(self.inpt, self.state, self.outpt) for device in self._devices]
        [process.init(robot=self) for process in self._processes]
        self.start_process()

    def reset(self, init_state, t):
        """process necessary to reset the robot without restarting"""
        [device.reset() for device in self._devices]


    def drive(self, inpt, timestamp):
        """drive the robot to the next state
        :param inpts: left, right wheel velocities
        """
        for device in self._devices:
            device.drive(inpt, timestamp)

    def sense(self):
        """generate the sensor reading
        :return output"""
        for device in self._devices:
            ret = device.sense()
            if ret is not None:
                self.outpt.update(ret)
        return self.outpt

    def observe_state(self):
        """get current state"""
        for device in self._devices:
            ret = device.observe_state()
            if ret is not None:
                self.state.update(ret)
        return self.state

    def process(self, t, *args, **kwargs):
        [process.process(t, *args, **kwargs) for process in self._processes]

    def open(self):
        [device.open() for device in self._devices]
        self.enable(True)

    def enable(self, enable):
        [device.enable(enable) for device in self._devices]

    def close(self):
        self.enable(False)
        [device.close() for device in self._devices]






