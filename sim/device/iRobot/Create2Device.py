from  pycreate2 import Create2
import time
from sim.device import DriveBase, SenseBase
from sim.typing import DefDict

dict(bumps_wheeldrops=float)


class Create2Device(DriveBase, SenseBase):
    def __init__(self, port, safety=True, *args, **keyword):
        super().__init__(*args, **keyword)
        self.config.on().set([True, True, False])
        self.device_name = 'Create 2'
        self.port = port
        self.safety = safety
        self.drive_space = DefDict(dict(wh_r=float, wh_l=float))
        self.sense_sapce = DefDict(dict(t=float))

    def open(self, *args, **kwargs):
        self.create = Create2(self.port)
        self.enable(True)

    def enable(self, enable, *args, **kwargs):
        if enable:
            self.create.start()
            if self.safety:
                self.create.safe()
            else:
                self.create.full()
        else:
            self.create.stop()

    def close(self, *args, **kwargs):
        self.enable(False)
        self.create.close()

    def drive(self, inpt, timestamp, *args, **kwargs):
        self.drive_space.update(inpt.vel().list())
        self.drive_space *= 10
        #print(self.drive_space)
        l, r = (int(i) for i in self.drive_space.list())
        self.create.drive_direct(r, l)

    def sense(self, *args, **kwargs):
        vals = self.create.get_sensors()
        print(vals)
        self.sense_sapce.set(vals)
        return self.sense_sapce


