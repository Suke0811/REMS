from sim.robots.RobotBase import RobotBase
from sim.utils.neural_network import NeuralNetwork
from sim.type.definitions import *

from sim.inputs import FileInput

class NopRobot(RobotBase):
    def __init__(self, file_inpt):
        super().__init__()
        self.run.name = 'Nop'
        self.run.realtime = False
        self.run.to_thread = True
        self.file_inpt: FileInput = file_inpt


    def drive(self, inpt, timestamp):
        self.inpt.data = self.file_inpt.get_inputs(self.inpt, timestamp)
        self.joint_space.data = self.ik(self.inpt)
        self.outpt = self.joint_space

        state = self.state.data.as_list()
        self.state.set_data(self.fk(self.outpt))
        self.calc_vel(pre_state=state, curr_state=self.state.data.as_list())

        return self.state.data_as(VEL_POS_3D).data.as_list()

    def sense(self):
        return self.outpt

    def observe_state(self):
        return self.state

    def calc_vel(self, pre_state, curr_state):
        dx = (curr_state[0] - pre_state[0]) / self.run.DT
        dy = (curr_state[1] - pre_state[1]) / self.run.DT
        dz = (curr_state[2] - pre_state[2]) / self.run.DT

        self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}
