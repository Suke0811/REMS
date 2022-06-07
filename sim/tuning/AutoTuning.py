from sim.tuning import TuningSystem
from sim.tuning import AutoTuner, AutoTuner2
from sim.utils.neural_network import NeuralNetwork
from sim.job_background.job_type import job_type
from sim.job_background.job_return_type import job_return_type
import numpy as np
HORIZON = 1
INIT_COV = 0.1 # 0.00001 for sim-to-kin for webots, 0.001 seems BEST FOR SIM-to-KIN PYbullet
LOAD_PARAM = False

class AutoTuning(TuningSystem):
    def __init__(self, target, ref, real_to_sim):
        super().__init__(target_robot=target, reference_robot=ref)
        # real to sim flag
        self.real_to_sim = real_to_sim
        # initialize cost
        self.cost = [10.0,1.0]
        # initialize robot class
        robot = self.target_robot
        self.states_sim = np.zeros((6,HORIZON))
        self.target_sim = np.zeros((6,HORIZON))
        self.inputs = np.zeros((3,HORIZON))
        N_horizon = self.states_sim.shape[1]
        # initialize the NN architecture
        NN_ARCHITECTURE = [
                {"input_dim": 6, "output_dim": 10, "activation": "leakyRelu"},
                {"input_dim": 10, "output_dim": 10, "activation": "leakyRelu"},
                {"input_dim": 10, "output_dim": 2, "activation": "linear"}
            ]

        # training objectives, is same shape as theta
        N_trainingObj = 2
        # Cv determines the weight given to each training objective (ex. is x more important than y), and Co determines
        # how quickly the auto-tuner will adapt (too fast may cause divergence)

        # weights also determine difference in magnitude between different sigma points
        weight_a = 0.01
        weight_c = 0.01
        if not real_to_sim:
            if LOAD_PARAM:
                try:
                    load_params = np.load('sim/controllers/NN_param.npy')
                except:
                    load_params = []
                    print('NO NEURAL NETWORK MODEL NAMED NN_param.npy exists')
            else:
                load_params = []
            NN = NeuralNetwork(NN_ARCHITECTURE, load_params)
            robot.init_NN(NN)
            self.auto_tuner = AutoTuner(INIT_COV, weight_a, weight_c, N_trainingObj, N_horizon, robot, self.cost)
        else:
            load_params = np.load('sim/controllers/NN_param.npy')
            NN2 = NeuralNetwork(NN_ARCHITECTURE, load_params)
            if LOAD_PARAM:
                try:
                    load_params = np.load('sim/controllers/NN_param.npy')
                except:
                    load_params = []
            NN = NeuralNetwork(NN_ARCHITECTURE, load_params)
            self.auto_tuner = AutoTuner2(INIT_COV, weight_a, weight_c, N_trainingObj, N_horizon, robot, self.cost, NN, NN2)
            self.target_robot.auto_tuner = self.auto_tuner

        self.time_count = 0
        self.update = True
        self.calcH2norm = True

    def process(self):
        """Process is called every time step"""
        # access to target/reference robot info
        target_state = self.target_robot.state.data.list()
        ref_state = self.ref_robot.state.data.list()
        inpt = self.ref_robot.inpt.data.list()

        # update state history
        self.states_sim = self._nparray_push(self.states_sim, ref_state)
        self.target_sim = self._nparray_push(self.target_sim, target_state)

        # update input history
        if not inpt:
            return 0

        self.inputs = self._nparray_push(self.inputs, inpt)

        self.states_sim_h2 = []
        self.target_sim_h2 = []

        for i in range(HORIZON):
            self.states_sim_h2 = np.hstack((self.states_sim_h2,self.states_sim[3:5,i]))
            self.target_sim_h2 = np.hstack((self.target_sim_h2,self.target_sim[3:5,i]))

        self.states_sim_h2 = self.states_sim_h2.reshape(2*HORIZON,1)
        self.target_sim_h2 = self.target_sim_h2.reshape(2*HORIZON,1)

        self.calculateH2Norm(self.states_sim_h2,self.target_sim_h2)
        self.target_robot.info.data = [self.h2_norm, self.h2_norm_x, self.h2_norm_y]
        print(self.auto_tuner.sigmas.flags)
        self.time_count += 1
        if self.time_count % HORIZON == 0 and self.time_count != 0:  # only run the tuning every Horizon
            #self.target_robot.state.data = self.ref_robot.state

            if self.update:
                return job_type(self.job)

    def _nparray_push(self, array, val):
        """this function mimic push queue mechanism (delete the first row and add new data at the end)"""
        return np.column_stack((np.delete(array,0,1), val))

    def job(self, **kwargs):
        if not self.real_to_sim:
            self.auto_tuner.update_parameters(self.states_sim, self.inputs)  # auto tuning update
        else:
            self.auto_tuner.update_parameters(self.states_sim, self.target_sim)
        return job_return_type(self.done, **kwargs)  # pack and go

    def done(self, **kwargs):
        self.calcH2norm = True

    def calculateH2Norm(self,states_sim_h2,target_sim_h2):
        self.h2_norm = np.linalg.norm((states_sim_h2-target_sim_h2) ** 2)
        self.h_act_list_x = []
        self.h_act_list_y = []
        self.y_k_list_x = []
        self.y_k_list_y = []
        for j in range(HORIZON):
            self.h_act_list_x.append(self.target_sim_h2[j * 2])
            self.h_act_list_y.append(self.target_sim_h2[j * 2 + 1])
            self.y_k_list_x.append(self.states_sim_h2[j * 2])
            self.y_k_list_y.append(self.states_sim_h2[j * 2 + 1])
        self.h_act_list_x = np.array(self.h_act_list_x).reshape(len(self.h_act_list_x), 1)
        self.h_act_list_y = np.array(self.h_act_list_y).reshape(len(self.h_act_list_y), 1)
        self.y_k_list_x = np.array(self.y_k_list_x).reshape(len(self.y_k_list_x), 1)
        self.y_k_list_y = np.array(self.y_k_list_y).reshape(len(self.y_k_list_y), 1)

        self.h2_norm_x = np.linalg.norm((self.h_act_list_x - self.y_k_list_x) ** 2)
        self.h2_norm_y = np.linalg.norm((self.h_act_list_y - self.y_k_list_y) ** 2)
