from sim.robots.RobotBase import RobotBase
from sim.utils.neural_network import NeuralNetwork
from sim.type.definitions import *
from sim.utils.tictoc import tictoc

class KinematicModel(RobotBase):
    def __init__(self):
        super().__init__()
        self.auto_tuner = None
        self.run.name = 'Kin'
        self.NN_normal = [1.0,1.0]

    def init_NN(self, NN: NeuralNetwork):
        # initialize the NN architecture
        self.NN = NN
        self.PARAMS = self.NN.Neural_to_Auto_Format(self.NN.params_values)

    def drive(self, inpt, timestamp):
        # TODO: PUT VELOCITY FUNCTION SAME AS OTHER FUNCTIONS HERE
        prev_joints = self.joint_space
        prev_joints = prev_joints.data_as(joint_pos(6)).data.as_list()

        self.inpt = inpt
        self.joint_space.data = self.ik(self.inpt)
        next_joints = self.joint_space.data.as_list()

        # add neural network
        # convert param values from auto tuner format into the neural network format
        # neural network input includes the current states and the control output u
        dj0 = (next_joints[0] - prev_joints[0]) / self.run.DT
        dj1 = (next_joints[1] - prev_joints[1]) / self.run.DT
        dj2 = (next_joints[2] - prev_joints[2]) / self.run.DT
        dj3 = (next_joints[3] - prev_joints[3]) / self.run.DT
        dj4 = (next_joints[4] - prev_joints[4]) / self.run.DT
        dj5 = (next_joints[5] - prev_joints[5]) / self.run.DT

        self.outpt = self.joint_space
        self.task_space.data = self.fk(self.joint_space)
        prev_state = self.state.data_as(POS_3D).data.as_list()

        self.state.data = self.task_space
        next_state = self.state.data_as(POS_3D).data.as_list()


        dx  = (next_state[0] - prev_state[0])/self.run.DT
        dy = (next_state[1] - prev_state[1])/self.run.DT
        dz = (next_state[2] - prev_state[2])/self.run.DT

        params_values_NN = self.NN.Auto_to_Neural_Format(self.PARAMS)
        self.NN.params_values = params_values_NN

        NN_input = np.array([dj0,dj1,dj2,dj3,dj4,dj5])
        NN_output, _ = self.NN.full_forward_propagation(np.transpose(NN_input.reshape(1, NN_input.shape[0])))
        dx = dx + NN_output[0]*self.run.DT
        dy = dy + NN_output[1]*self.run.DT

        self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}

        return self.state.data_as(VEL_POS_3D).data.as_list()

    def sense(self):
        return self.outpt

    def observe_state(self):
        return self.state
