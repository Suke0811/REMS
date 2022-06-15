from sim.robots.DeviceBase import DeviceBase, DefDict
import ray

@ray.remote
class DeviceActor(DeviceBase):
    def __init__(self, device):
        self.device = device()

    def init(self):
        self.open()
        self.enable(True)

    def open(self):
        self.device.open()

    def close(self):
        self.device.close()

    def enable(self, enable):
        self.device.enable(enable)

    def drive(self, inpt: DefDict, timestamp):
        self.device.drive(inpt)

    def sense(self):
        return self.device.sense()

    def observe_state(self):
        return self.device.observe_state()
