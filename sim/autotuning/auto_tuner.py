import numpy as np
from sim.constants import ROBOT, ENVIRONMENT
from sim.controllers import DifferentialDrive
from numpy import linalg as la
from sim.utils.neural_network import NeuralNetwork
__author__ = "Alexander Schperberg"
__email__ = "aschperberg28@g.ucla.edu"
__copyright__ = "MERL"
__date__ = "March, 2022"
"""SHARING OF CODE PROHIBITED, PLEASE EMAIL FOR PERMISSION"""
"""This class constructs an auto-tuner formulation for finding kinematic constants for the webots simulation. The goal
is to accomplish real to sim. The Auto-tuning method used can be found in the paper, "Kalman Filter for Controller 
Calibration" by Marcel Menner, Karl Berntorp, and Stefano Di Cairano."""

class AutoTuner():
    def __init__(self, initial_theta, Cv, Co, P_theta, weight_a, weight_c, Ntheta, NtrainingObj, Robot_class, neural_network=None):
        # provide the covariance matrix Cv and Co, where Cv determines the relative importance of the vector-valued
        # objective vector yk, and Co determines the aggressiveness of the adaptation. These matrices will be 3x3.
        self.Cv = Cv
        self.Co = Co
        # provide the covariance matrix for tuning the gains (N_theta x N_theta)
        self.P_theta = P_theta
        # provide number of control variables
        self.N_theta = Ntheta
        # provide number of states (we assume number of states corresponds to size of the training objective h)
        self.N_trainingObj = NtrainingObj
        # initialize the given initial weights which affect the sigma points
        self.w_a0 = weight_a
        self.w_c0 = weight_c
        # create array of weights, based on the given initial weight (using definition from the paper)
        self.w_a = np.zeros((self.N_theta * 2, 1))
        self.w_c = np.zeros((self.N_theta * 2, 1))
        for i in range(self.w_a.shape[0]):
            self.w_a[i] = (1.0 - self.w_a0) / (2.0 * self.N_theta)
            self.w_c[i] = (1.0 - self.w_c0) / (2.0 * self.N_theta)
        # input epsilon (used for projecting the semi-definite matrix, if required)
        self.epsi = 0.001
        # create array for the sigma points
        self.sigmas = np.zeros((self.N_theta, self.N_theta * 2 + 1))
        self.theta = initial_theta
        self.Robot = Robot_class
        self.use_NN = False
        # impose importance of cost on each variable
        self.cost = [1.0,1.0,1.0]
        self.NN = neural_network

    def update_parameters(self, states_sim, inputs, dt):
        self.states_sim, self.inputs, self.dt = states_sim, inputs, dt
        self.N_horizon = self.states_sim.shape[1]
        # initialize h (or y in equation 15e of the paper), note we include the initial sigma result as well
        self.h_est = np.zeros((self.N_horizon * self.N_trainingObj, self.N_theta * 2 + 1))
        # initialize W (error in dynamics at the joint space)
        self.W_est = np.zeros((self.N_trainingObj, self.N_horizon))
        # create initial y (considered the nominal values)
        self.y_k = np.zeros((self.N_horizon * self.N_trainingObj,))
        for i in range(self.N_horizon):
            nominal_values = np.array([self.states_sim[0,i]*self.cost[0],self.states_sim[1,i]*self.cost[1],self.states_sim[3,i]*self.cost[2]]).reshape(self.N_trainingObj,)
            self.y_k[i * self.N_trainingObj:i * self.N_trainingObj + self.N_trainingObj] = nominal_values
        self.y_k =self.y_k.reshape(self.N_trainingObj*self.N_horizon,1)
        # we first calculate our sigma points
        self.calcSigmas()
        # calculate h over the trajectory
        self.calcH_est()
        # initiate the prediction of theta and h
        self.prediction_theta()
        self.prediction_h()
        # initiate propagation
        self.propagation()
        # initiate the update step
        self.update()

    def calcSigmas(self):
        # perform cholesky decomposition on the current P_theta covariance matrix, ensure it is positive definite,
        # if not, force it to be positive definite through several operations found in the positiveDef function.
        try:
            P_theta_chol = np.linalg.cholesky(np.real(self.P_theta))
        except:
            P_theta_pos_def = self.positiveDef(self.P_theta)
            P_theta_chol = np.linalg.cholesky(P_theta_pos_def)
        # calculate the sigma points
        # initial sigma point is simply the current 'theta' values (or current gains)
        self.sigmas[:,0] = self.theta.reshape(self.N_theta,)
        for i in range(self.N_theta):
            sigma_pos = self.theta + np.sqrt(self.N_theta / (1 - self.w_a0)) * P_theta_chol[:, i].reshape(self.N_theta, 1)
            self.sigmas[:, i+1] = sigma_pos.reshape(self.N_theta, )
            sigma_neg = self.theta - np.sqrt(self.N_theta / (1 - self.w_a0)) * P_theta_chol[:, i].reshape(self.N_theta, 1)
            self.sigmas[:, i+self.N_theta+1] = sigma_neg.reshape(self.N_theta, )

    def positiveDef(self, A):
        B = (A + A.T) / 2
        _, s, V = la.svd(B)
        H = np.dot(V.T, np.dot(np.diag(s), V))
        A2 = (B + H) / 2
        A3 = (A2 + A2.T) / 2
        if self.isPD(A3):
            return A3
        spacing = np.spacing(la.norm(A))
        I = np.eye(A.shape[0])
        k = 1
        while not self.isPD(A3):
            mineig = np.min(np.real(la.eigvals(A3)))
            A3 += I * (-mineig * k ** 2 + spacing)
            k += 1
        return A3

    def isPD(self,B):
        """Returns true when input is positive-definite, via Cholesky"""
        try:
            _ = la.cholesky(B)
            return True
        except la.LinAlgError:
            return False

    def calcH_est(self):
        for i in range(self.N_theta * 2 + 1):
            # get the current sigma value
            sigma_curr = self.sigmas[:, i]
            # initialize robot parameters to current sigma points
            self.Robot.PARAMS = sigma_curr
            # get initial states received from the simulated robot
            state_sim_cur = self.states_sim[:, 0]
            h_init = np.array([self.states_sim[0,0]*self.cost[0],self.states_sim[1,0]*self.cost[1],self.states_sim[3,0]*self.cost[2]]).reshape(self.N_trainingObj,)
            self.h_est[0 * self.N_trainingObj:0 * self.N_trainingObj + self.N_trainingObj, i] = h_init.reshape(self.N_trainingObj,)
            cur_vel_x = (self.states_sim[0,1]-self.states_sim[0,0])/self.dt
            cur_vel_y = (self.states_sim[1,1]-self.states_sim[1,0])/self.dt

            for j in range(self.N_horizon-1):
                # get output of neural network based on current state
                # convert param values from auto tuner format into the neural network format
                params_values_NN = self.NN.Auto_to_Neural_Format(self.Robot.PARAMS)
                self.NN.params_values = params_values_NN
                # neural network input includes the current states and the control output u
                NN_input = np.array([state_sim_cur[0],state_sim_cur[1],state_sim_cur[3],cur_vel_x,cur_vel_y,self.inputs[0,j+1],self.inputs[1,j+1]])
                # get output based on current control parameters from auto tuner
                output, _ = self.NN.full_forward_propagation(np.transpose(NN_input.reshape(1,NN_input.shape[0])))
                self.Robot.NN_OUTPUT = output

                # get initial inputs received from the simulated reference
                inpts_cur = self.inputs[:, j+1]
                state_sim_next = self.Robot.next_state(state_sim_cur,inpts_cur)
                cur_vel_x = (state_sim_next[0]-state_sim_cur[0])/self.dt
                cur_vel_y = (state_sim_next[1]-state_sim_cur[1])/self.dt


                state_sim_cur = state_sim_next
                # we overwrite the theta value as we don't want to propagate theta
                #state_sim_cur[2] = self.states_sim[2,j+1]
                h_cost = np.array([state_sim_cur[0]*self.cost[0],state_sim_cur[1]*self.cost[1],state_sim_cur[3]*self.cost[2]]).reshape(self.N_trainingObj,)
                self.h_est[(j + 1) * self.N_trainingObj:(j + 1) * self.N_trainingObj + self.N_trainingObj, i] = h_cost.reshape(self.N_trainingObj,)

    def prediction_theta(self):
        # predict the new sigma values (eqn 15g)
        # get initial sigma
        sig_0 = self.sigmas[:,0].reshape(self.N_theta,1)
        # calculate theta_hats (which size is equal to N_theta x N_theta*2)
        # calculate initial sig_hat
        sig_Hat0 = self.w_a0 * sig_0
        # calculate sig_hat (summation)
        for i in range(self.N_theta * 2):
            sig_Hat0 = sig_Hat0 + self.w_a[i] * self.sigmas[:, i+1].reshape(self.N_theta, 1)
        # calculate covariance P (eqn 15f)
        # calculate initial P
        P_0 = self.Co + self.w_c0 * (sig_0 - sig_Hat0) * (np.transpose(sig_0 - sig_Hat0))
        # calculate P_k_pre
        for i in range(self.N_theta * 2):
            P_0 = P_0 + self.w_c[i] * (self.sigmas[:, i+1].reshape(self.N_theta, 1) - sig_Hat0) * (
                np.transpose(self.sigmas[:, i+1].reshape(self.N_theta, 1) - sig_Hat0))

        self.P_k_pre = P_0
        self.sig_hat = sig_Hat0

    def prediction_h(self):
        # calculate the initial y (h(sigma0))
        y_hat_0 = self.w_a0*self.h_est[:,0].reshape(self.N_trainingObj*self.N_horizon,1)
        for i in range(self.N_theta):
            y_hat0 = y_hat_0 + self.w_a[i]*self.h_est[:,i+1].reshape(self.N_trainingObj*self.N_horizon,1)

        self.y_hat = y_hat0

    def propagation(self):
        # calculate the matrix S (based on our y_hat calculation)
        S_0 = self.Cv + self.w_c0 * (self.h_est[:, 0].reshape(self.N_trainingObj*self.N_horizon,1) - self.y_hat) * \
              (np.transpose(self.h_est[:, 0].reshape(self.N_trainingObj*self.N_horizon,1) - self.y_hat))
        for i in range(self.N_theta):
            S_0 = S_0 + self.w_c[i] * (self.h_est[:, i + 1].reshape(self.N_trainingObj*self.N_horizon,1) - self.y_hat) *\
                (np.transpose(self.h_est[:, i + 1].reshape(self.N_trainingObj*self.N_horizon,1) - self.y_hat))

        self.S = S_0

        # calculate the matrix Csz
        sigma_0 = self.sigmas[:,0].reshape(self.N_theta,1)
        Csz_0 = self.w_c0 * (sigma_0 - self.sig_hat) * \
                (np.transpose(self.h_est[:,0].reshape(self.N_trainingObj*self.N_horizon,1) - self.y_hat))

        for i in range(self.N_theta):
            Csz_0 = Csz_0 + self.w_c[i] * (self.sigmas[:,i+1].reshape(self.N_theta,1) - self.sig_hat) * \
                  (np.transpose(self.h_est[:,i+1].reshape(self.N_trainingObj*self.N_horizon,1)  - self.y_hat))

        self.Csz = Csz_0

    def update(self):
        # calculate the Kalman gain
        self.K = np.matmul(self.Csz, np.linalg.inv(self.S)) # TODO: use least square operator than inv
        # update the covariance of sigma
        self.P_theta = self.P_k_pre - np.matmul(np.matmul(self.K, self.S), np.transpose(self.K))
        # update the gains
        self.theta = self.sig_hat + np.matmul(self.K, self.y_k - self.y_hat)
        # initialize robot parameters to finalized parameter solution
        self.Robot.PARAMS = self.theta.reshape(self.theta.shape[0], )
        params_values_NN = self.NN.Auto_to_Neural_Format(self.Robot.PARAMS)
        self.NN.params_values = params_values_NN

    def calcH2Norm(self, states_sim, inputs):
        # create initial y (considered the nominal values)
        self.y_k = np.zeros((self.N_horizon * self.N_trainingObj,))
        for i in range(self.N_horizon):
            nominal_values = np.array([states_sim[0, i], states_sim[1, i], states_sim[3, i]]).reshape(self.N_trainingObj, )
            self.y_k[i * self.N_trainingObj:i * self.N_trainingObj + self.N_trainingObj] = nominal_values
        self.y_k = self.y_k.reshape(self.N_trainingObj * self.N_horizon, 1)
        # Calculate the H2 Norm
        self.h_act = np.zeros((self.y_k.shape[0], self.y_k.shape[1]))

        # get initial states received from the simulated robot
        state_sim_cur = states_sim[:, 0]
        h_init = np.array([state_sim_cur[0], state_sim_cur[1], state_sim_cur[3]]).reshape(self.N_trainingObj, )

        self.h_act[0 * self.N_trainingObj:0 * self.N_trainingObj + self.N_trainingObj,0] = h_init.reshape(self.N_trainingObj, )
        cur_vel_x = (states_sim[0, 1] - states_sim[0, 0]) / self.dt
        cur_vel_y = (states_sim[1, 1] - states_sim[1, 0]) / self.dt

        for j in range(self.N_horizon - 1):
            # get initial inputs received from the simulated reference
            inpts_cur = inputs[:, j + 1]

            params_values_NN = self.NN.Auto_to_Neural_Format(self.Robot.PARAMS)
            self.NN.params_values = params_values_NN
            # neural network input includes the current states and the control output u
            NN_input = np.array([state_sim_cur[0], state_sim_cur[1], state_sim_cur[3], cur_vel_x, cur_vel_y, inputs[0, j + 1], inputs[1, j + 1]])
            # get output based on current control parameters from auto tuner
            output, _ = self.NN.full_forward_propagation(
                np.transpose(NN_input.reshape(1, NN_input.shape[0])))
            self.Robot.NN_OUTPUT = output

            state_sim_next = self.Robot.next_state(state_sim_cur, inpts_cur)
            cur_vel_x = (state_sim_next[0]-state_sim_cur[0])/self.dt
            cur_vel_y = (state_sim_next[1]-state_sim_cur[1])/self.dt

            state_sim_cur = state_sim_next
            # we overwrite the theta value as we don't want to propagate theta
            #state_sim_cur[2] = states_sim[2, j + 1]
            h_cost = np.array([state_sim_cur[0], state_sim_cur[1], state_sim_cur[3]]).reshape(self.N_trainingObj, )
            self.h_act[(j + 1) * self.N_trainingObj:(j + 1) * self.N_trainingObj + self.N_trainingObj,0] = h_cost.reshape(self.N_trainingObj, )

        # we take H2 norm of the propagation using kinematic model with the reference model
        self.h2_norm = np.linalg.norm((self.h_act - self.y_k) ** 2)
        self.h_act_list_x = []
        self.h_act_list_y = []
        self.h_act_list_th = []
        self.y_k_list_x = []
        self.y_k_list_y = []
        self.y_k_list_th = []
        for j in range(self.N_horizon - 1):
            self.h_act_list_x.append(self.h_act[j * 3])
            self.h_act_list_y.append(self.h_act[j * 3 + 1])
            self.h_act_list_th.append(self.h_act[j * 3 + 2])
            self.y_k_list_x.append(self.y_k[j * 3])
            self.y_k_list_y.append(self.y_k[j * 3 + 1])
            self.y_k_list_th.append(self.y_k[j * 3 + 2])
        self.h_act_list_x = np.array(self.h_act_list_x).reshape(len(self.h_act_list_x), 1)
        self.h_act_list_y = np.array(self.h_act_list_y).reshape(len(self.h_act_list_y), 1)
        self.h_act_list_th = np.array(self.h_act_list_th).reshape(len(self.h_act_list_th), 1)
        self.y_k_list_x = np.array(self.y_k_list_x).reshape(len(self.y_k_list_x), 1)
        self.y_k_list_y = np.array(self.y_k_list_y).reshape(len(self.y_k_list_y), 1)
        self.y_k_list_th = np.array(self.y_k_list_th).reshape(len(self.y_k_list_th), 1)

        self.h2_norm_x = np.linalg.norm((self.h_act_list_x - self.y_k_list_x) ** 2)
        self.h2_norm_y = np.linalg.norm((self.h_act_list_y - self.y_k_list_y) ** 2)
        self.h2_norm_th = np.linalg.norm((self.h_act_list_th - self.y_k_list_th) ** 2)


# for testing and debugging
if __name__ == '__main__':
    # initialize robot class
    robot = DifferentialDrive()
    robot.DT = 0.1
    states_real = np.random.sample(size=(4,10))
    states_real[0:2,:] = states_real[0:2,:]*10
    states_sim = np.random.sample(size=(4,10))
    states_sim[0:2, :] = states_sim[0:2, :]*10
    inputs = np.ones((2,10))
    inputs[1,:] = inputs[1,:]*-1
    N_horizon = states_real.shape[1]
    # our control parameter theta will consist of 3 components, wheel radius, robot length, and servo speed
    N_theta = 3
    # training objectives, is same shape as theta
    N_trainingObj = 3
    # initial theta
    initial_theta = np.array([robot.WHEEL_RADIUS,robot.LENGTH,robot.SERVO_SPEED]).reshape(3,1)
    # weights also determine difference in magnitude between different sigma points
    weight_a = 0.01
    weight_c = 0.01
    # get dt of the robot
    dt = robot.DT
    NN_ARCHITECTURE = [
        {"input_dim": 7, "output_dim": 20, "activation": "relu"},
        {"input_dim": 20, "output_dim": 3, "activation": "sigmoid"},
    ]

    NN = NeuralNetwork(NN_ARCHITECTURE)
    initial_theta = NN.Neural_to_Auto_Format(NN.params_values)
    N_theta = initial_theta.shape[0]

    # Cv determines the weight given to each training objective (ex. is x more important than y), and Co determines
    # how quickly the auto-tuner will adapt (too fast may cause divergence)
    Cv = np.eye(N_horizon * N_trainingObj)
    Co = np.eye(N_theta) * 0.0001
    P_theta = Co
    auto_tuner = AutoTuner(initial_theta,Cv,Co,P_theta,weight_a,weight_c,N_theta,N_trainingObj,robot,NN)
    import time
    time_vec = []
    for i in range(100):
        start = time.time()
        auto_tuner.update_parameters(states_sim,inputs,dt)
        end = time.time() - start
        time_vec.append(end)

    print(sum(time_vec)/len(time_vec))