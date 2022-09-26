from rems.device import SenseBase
from rems.device.imu.Imu_interface import imu
from rems.typing import DefDict


class Imu(SenseBase):
    def __init__(self):
        self.imu = None
        definition = {'acc.x': 0.0, 'acc.y': 0.0, 'acc.z': 0.0,
                      'vel.x': 0.0, 'vel.y': 0.0, 'vel.z': 0.0,
                      'rot.x': 0.0, 'rot.y': 0.0, 'rot.z': 0.0}
        self.imu_outpt = DefDict(definition, prefixes=['acc', 'vel', 'rot'], suffixes=['x', 'y', 'z'])
        super().__init__()

    def init(self):
        self.imu = imu()
        self.open()
        self.enable(True)

    def open(self):
        try:
            self.imu.calibBias()  # Calibrate bias here
        except RuntimeError:
            raise ConnectionError('IMU is not Properly connected')

    def close(self):
        self.imu.close()

    def enable(self, enable):
        if enable:
            self.imu.node.resume()

    def sense(self):
        self.imu_outpt.set(self.imu.getData())
        return self.imu_outpt

if __name__ == '__main__':
    i = Imu()
    i.init()
    print(i.sense())
    i.close()
