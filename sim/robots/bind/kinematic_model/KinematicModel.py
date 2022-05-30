from sim.robots.RobotBase import RobotBase
from sim.utils.neural_network import NeuralNetwork
from sim.type.definitions import *

class KinematicModel(RobotBase):
    def __init__(self):
        super().__init__()
        self.auto_tuner = None
        self.run.name = 'Kin'

    def init_NN(self, NN: NeuralNetwork):
        # initialize the NN architecture
        self.NN = NN
        self.PARAMS = self.NN.Neural_to_Auto_Format(self.NN.params_values)

    def drive(self, inpt, timestamp):
        self.inpt = inpt
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
        params_values_NN = self.NN.Auto_to_Neural_Format(self.PARAMS)
        self.NN.params_values = params_values_NN
        inpt = self.inpt.data.as_list()
        NN_input = np.array([(inpt[0]+2.0)/10.0,(inpt[1]+2.0)/10.0,(dx+2.0)/10.0,(dy+2.0)/10.0])
        NN_output, _ = self.NN.full_forward_propagation(np.transpose(NN_input.reshape(1, NN_input.shape[0])))
        dx = dx + NN_output[0][0] + 1.0
        dy = dy + NN_output[1][0] - 1.0

        self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}
