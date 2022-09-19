from sim.robots import RobotBase
from sim.device.iRobot.Create2Device import Create2Device
from sim.typing import DefDict
from sim.utils import time_str


class JacobianModel(RobotBase):
    DEVICE_LIST = []
    def __init__(self, joint_to_task=True,*args, **kwargs):
        super().__init__(*args, **kwargs)
        self.run.DT = 0.05
        self.run.name = 'Model'
        self.joint_to_task = joint_to_task

    def drive(self, inpt, timestamp):
        if self.joint_to_task:
            jb = self.jb(None).ndarray()
            n_state = self.state.ndtall() + jb @ inpt.ndtall() * self.run.DT
        else:
            n_state = self.state.ndtall() + inpt.ndtall() * self.run.DT
        self.state.update(n_state.flatten())
