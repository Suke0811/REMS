from sim.robots.RobotBase import RobotBase
from sim.utils.neural_network import NeuralNetwork
from sim.typing.definitions import *

class KinematicModel(RobotBase):
    def __init__(self):
        super().__init__()
        self.auto_tuner = None
        self.run.name = 'Kin'
        self.NN_normal = [1.0,1.0]
        self.NN = None
        self.PARAMS = None

    def init_NN(self, NN: NeuralNetwork):
        # initialize the NN architecture
        self.NN = NN
        self.PARAMS = self.NN.Neural_to_Auto_Format(self.NN.params_values)

    def drive(self, inpt, timestamp):
        # TODO: PUT VELOCITY FUNCTION SAME AS OTHER FUNCTIONS HERE
        prev_joints = self.joint_space.ndarray()

        self.inpt.set(inpt)
        self.joint_space.set(self.ik(self.inpt))
        next_joints = self.joint_space.ndarray()

        # add neural network
        # convert param values from auto tuner format into the neural network format
        # neural network input includes the current states and the control output u

        self.outpt.set(self.joint_space)
        self.task_space.set(self.fk(self.joint_space))
        prev_state = self.state.get(POS_3D).ndarray()
        self.state.set(self.task_space)
        next_state = self.state.get(POS_3D).ndarray()
        d_X = (next_state - prev_state)/self.run.DT

        params_values_NN = self.NN.Auto_to_Neural_Format(self.PARAMS)
        self.NN.params_values = params_values_NN

        NN_input = (next_joints-prev_joints) / self.run.DT
        NN_output, _ = self.NN.full_forward_propagation(np.transpose(NN_input.reshape(1, NN_input.shape[0])))
        NN_output*=self.run.DT
        d_X += np.append(NN_output, 0.0)

        self.state.get(VEL_POS_3D).set(d_X)
        return self.state.get(VEL_POS_3D).list()

    def sense(self):
        return self.outpt

    def observe_state(self):
        return self.state
