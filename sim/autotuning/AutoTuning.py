from sim.controllers import TuningSystem, AutoTuner
from sim.utils.neural_network import NeuralNetwork
from sim.autotuning.AutoTuningBackground import *
import numpy as np
HORIZON = 10
INIT_COV = 0.001

class AutoTuning(TuningSystem):
    def __init__(self, target, ref):
        super().__init__(target_robot=target, reference_robot=ref)
        # initialize robot class
        robot = self.target_robot
        states_real = np.random.sample(size=(4,HORIZON))
        states_real[0:2,:] = states_real[0:2,:]*10
        self.states_sim = np.random.sample(size=(4,HORIZON))
        self.states_sim[0:2, :] = self.states_sim[0:2, :]*10
        self.inputs = np.ones((2,HORIZON))
        self.inputs[1,:] = self.inputs[1,:]*-1
        N_horizon = states_real.shape[1]
        # our control parameter theta will consist of 3 components, wheel radius, robot length, and servo speed
        # initialize the NN architecture
        NN_ARCHITECTURE = [
                {"input_dim": 7, "output_dim": 10, "activation": "relu"},
                {"input_dim": 10, "output_dim": 10, "activation": "relu"},
                {"input_dim": 10, "output_dim": 3, "activation": "linear"}
            ]
        NN = NeuralNetwork(NN_ARCHITECTURE)
        initial_theta = NN.Neural_to_Auto_Format(NN.params_values)
        N_theta = initial_theta.shape[0]

        # training objectives, is same shape as theta
        N_trainingObj = 3
        # Cv determines the weight given to each training objective (ex. is x more important than y), and Co determines
        # how quickly the auto-tuner will adapt (too fast may cause divergence)
        Cv = np.eye(N_horizon * N_trainingObj)
        self.Co = np.eye(N_theta)*INIT_COV
        P_theta = self.Co
        # weights also determine difference in magnitude between different sigma points
        weight_a = 0.01
        weight_c = 0.01
        self.auto_tuner = AutoTuner(initial_theta,Cv,self.Co,P_theta,weight_a,weight_c,N_theta,N_trainingObj,robot,NN)
        self.time_count = 0
        self.update = True
        self.calcH2norm = False

    def process(self):
        """Process is called every time step"""

        # access to target/reference robot info
        target_state = self.target_robot.state
        ref_state = self.ref_robot.state
        inpt = self.ref_robot.inpt

        # update state history
        self.states_sim = self._nparray_push(self.states_sim, ref_state)
        # update input history
        if not inpt:
            return 0
        self.inputs = self._nparray_push(self.inputs, inpt)
        if self.time_count != 0 and self.time_count >= HORIZON and self.calcH2norm:
            self.auto_tuner.calcH2Norm(self.states_sim,self.inputs)
            self.target_robot.info = [self.auto_tuner.h2_norm, self.auto_tuner.h2_norm_x, self.auto_tuner.h2_norm_y, self.auto_tuner.h2_norm_th]

            #if self.auto_tuner.h2_norm_th <= 0.08:
            #    self.auto_tuner.cost[0] = 10.0
            #    self.auto_tuner.cost[1] = 10.0
            #    self.auto_tuner.cost[2] = 1.0
            #    self.update = False

        if self.time_count % HORIZON == 0 and self.time_count != 0:  # only run the tuning every Horizon
            self.target_robot.state = self.ref_robot.state
            if self.update:
                return job_type(self.job)

        self.time_count += 1

    def _nparray_push(self, array, val):
        """this function mimic push queue mechanism (delete the first row and add new data at the end)"""
        return np.column_stack((np.delete(array,0,1), val))

    def job(self, **kwargs):
        self.auto_tuner.update_parameters(self.states_sim, self.inputs, self.target_robot.DT)  # auto tuning update
        return job_return_type(self.done, **kwargs)  # pack and go

    def job2(self, theta):
        # something
        return job_return_type(self.done)  # pack and go

    def done(self, **kwargs):
        self.calcH2norm = True
        #t = kwargs.get("count")  # you can extract variables
        #print("done: {}".format(t)) # print to show it is working
