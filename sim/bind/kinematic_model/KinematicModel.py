from sim.robots.RobotBase import RobotBase
from sim.typing.definitions import *
from sim.utils import tictoc

class KinematicModel(RobotBase):
    def __init__(self):
        super().__init__()
        self.run.name = 'Kin'

    def drive(self, inpt, timestamp):
        self.inpt.set(inpt)
        self.joint_space.set(self.ik(self.inpt))
        self.outpt = self.joint_space
        self.task_space.set(self.fk(self.joint_space))
        prev_state = self.state.format(POS_3D).list()

        self.state.set(self.task_space)
        next_state = self.state.format(POS_3D).list()

        dx = (next_state[0] - prev_state[0])/self.run.DT
        dy = (next_state[1] - prev_state[1])/self.run.DT
        dz = (next_state[2] - prev_state[2])/self.run.DT
        self.state.set({'d_x': dx, 'd_y': dy, 'd_z': dz})

    def sense(self):
        return self.outpt

    def observe_state(self):
        return self.state
