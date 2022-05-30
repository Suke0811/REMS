from sim.robots.RobotBase import RobotBase
from sim.utils.neural_network import NeuralNetwork
from sim.type.definitions import *

class KinematicModel(RobotBase):
    def __init__(self):
        super().__init__()
        self.auto_tuner = None
        self.run.name = 'Kin'
        self.NN_normal = [1.0,1.0]
        self.moving_average_dx = []
        self.moving_average_dy = []
        self.moving_average_total = 5

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

        inpt = self.inpt.data.as_list()

        # add neural network
        # convert param values from auto tuner format into the neural network format
        # neural network input includes the current states and the control output u
        dj0 = (next_joints[0] - prev_joints[0]) / self.run.DT
        dj1 = (next_joints[1] - prev_joints[1]) / self.run.DT
        dj2 = (next_joints[2] - prev_joints[2]) / self.run.DT
        dj3 = (next_joints[3] - prev_joints[3]) / self.run.DT
        dj4 = (next_joints[4] - prev_joints[4]) / self.run.DT
        dj5 = (next_joints[5] - prev_joints[5]) / self.run.DT
        dj = [dj0, dj1, dj2, dj2, dj3, dj4, dj5]

        self.outpt = self.joint_space
        #self.task_space.data = self.fk(self.joint_space)
        #prev_state = self.state.data_as(POS_3D).data.as_list()

        #self.state.data = self.task_space
        #next_state =  self.state.data_as(POS_3D).data.as_list()

        state = self.state
        self.state.set_data(self.fk(self.outpt))
        self.calc_vel(pre_state=state, curr_state=self.state, dj=dj)
        #self.task_space.data = self.fk(self.joint_space)

        #dx  = (next_state[0] - prev_state[0])/self.run.DT
        #dy = (next_state[1] - prev_state[1])/self.run.DT
        #dz = (next_state[2] - prev_state[2])/self.run.DT

        #params_values_NN = self.NN.Auto_to_Neural_Format(self.PARAMS)
        #self.NN.params_values = params_values_NN

        #NN_input = np.array([(inpt[0]+2.0)/10.0,(inpt[1]+2.0)/10.0,(dj0+2.0)/10,(dj1+2.0)/10,(dj2+2.0)/10,(dj3+2.0)/10,(dj4+2.0)/10,(dj5+2.0)/10])
        #NN_output, _ = self.NN.full_forward_propagation(np.transpose(NN_input.reshape(1, NN_input.shape[0])))
        #dx = dx + NN_output[0]*self.run.DT + 1.0
        #dy = dy + NN_output[1]*self.run.DT - 1.0

        #self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}

        return self.state.data_as(VEL_POS_3D).data.as_list()

    def sense(self):
        return self.outpt

    def observe_state(self):
        return self.state

    def calc_vel(self, pre_state, curr_state, dj):
        prev_state = pre_state.data_as(POS_3D).data.as_list()
        self.state.data = self.task_space
        next_state = curr_state.data_as(POS_3D).data.as_list()
        dx = (next_state[0] - prev_state[0]) / self.run.DT
        dy = (next_state[1] - prev_state[1]) / self.run.DT
        dz = (next_state[2] - prev_state[2]) / self.run.DT
        params_values_NN = self.NN.Auto_to_Neural_Format(self.PARAMS)
        self.NN.params_values = params_values_NN
        #NN_input = np.array([dx,dy,dj[0],dj[1],dj[2],dj[3],dj[4],dj[5]])
        inpt = self.inpt.data.as_list()
        NN_input = np.array([(inpt[0]+2.0)/10.0,(inpt[1]+2.0)/10.0,(dx+2.0)/10.0,(dy+2.0)/10.0])
        NN_output, _ = self.NN.full_forward_propagation(np.transpose(NN_input.reshape(1, NN_input.shape[0])))
        dx = dx + NN_output[0][0] + 1.0
        dy = dy + NN_output[1][0] - 1.0

        #self.moving_average_dx.append(dx)
        #self.moving_average_dy.append(dy)

        #if len(self.moving_average_dx) > self.moving_average_total:
        #    self.moving_average_dx.pop(0)
        #    self.moving_average_dy.pop(0)
        #    dx = sum(self.moving_average_dx) / len(self.moving_average_dx)
        #    dy = sum(self.moving_average_dy) / len(self.moving_average_dy)

        self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}
