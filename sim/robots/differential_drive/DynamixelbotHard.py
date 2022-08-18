from sim.robots import RobotBase
from sim.device.Dynamixel.Dynamixel import Dynamixel
from sim.typing import DefDict


ID_LISTs = [ 2, 1]


class DynamixelbotHard(RobotBase):
    def __init__(self, port, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.add_device(Dynamixel, id_lists=ID_LISTs, slave_ids=None, device_port=port)
        self.run.name = 'Hard'
        self.run.DT = 0.02


    def init(self, *args, **kwargs):
        super().init(*args, **kwargs)

    def drive(self, inpt, timestamp):
        inpt /= 3
        super(DynamixelbotHard, self).drive(inpt, timestamp)


if __name__ == '__main__':

    motor = DefDict({
        "left wheel motor": dict(pos=float('inf'), vel=float, acc=float, on=bool, pid=list),
        "right wheel motor": dict(pos=float('inf'), vel=float, acc=float, on=bool, pid=list),
    })
    import ray
    #ray.init(local_mode=True)
    d = DynamixelbotHard('COM3')
    d.init()
    d.open()
