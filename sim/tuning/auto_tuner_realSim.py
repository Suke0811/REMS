import copy
import numpy as np
from numpy import linalg as la
__author__ = "Alexander Schperberg"
__email__ = "aschperberg28@g.ucla.edu"
__copyright__ = "MERL"
__date__ = "March, 2022"
"""SHARING OF CODE PROHIBITED, PLEASE EMAIL FOR PERMISSION"""
"""This class constructs an auto-tuner formulation for finding kinematic constants for the webots simulation. The goal
is to accomplish real to sim. The Auto-tuning method used can be found in the paper, "Kalman Filter for Controller 
Calibration" by Marcel Menner, Karl Berntorp, and Stefano Di Cairano."""

class AutoTuner2():
    def __init__(self, init_cov, weight_a, weight_c, NtrainingObj, N_horizon, Robot_class, cost, NN, NN2):
        # provide the covariance matrix Cv and Co, where Cv determines the relative importance of the vector-valued
        # objective vector yk, and Co determines the aggressiveness of the adaptation. These matrices will be 3x3.
        self.NN = NN
        self.NN2 = NN2
        self.Robot = Robot_class
        self.theta = NN.Neural_to_Auto_Format(self.NN.params_values)
        self.N_theta = self.theta.shape[0]
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
        #self.theta = initial_theta
        self.Cv = np.eye(N_horizon * NtrainingObj)
        self.Co = np.eye(self.N_theta)*init_cov
        self.P_theta = self.Co
        # impose importance of cost on each variable
        self.cost = cost
        self.N_horizon = N_horizon

    def update_parameters(self, states_real, states_sim, inpts):
        # ref_states are the reference states in task space (assume list of x,y and z, and dx,dy and dz) from simulator or real robot
        # joint_states are the reference joint states in joint space from the reference state trajectory (assume list of 12 -- joint angles and joint velocities)
        if self.theta.shape[1] == self.N_horizon*2:
            self.theta = self.theta[:,0].reshape(self.theta.shape[0],1)

        self.states_real, self.states_sim, self.inpts = states_real, states_sim, inpts
        self.N_horizon = self.states_real.shape[1]
        # initialize h (or y in equation 15e of the paper), note we include the initial sigma result as well
        self.h_est = np.zeros((self.N_horizon * self.N_trainingObj, self.N_theta * 2 + 1))
        # create initial y (considered the nominal values)
        self.y_k = np.zeros((self.N_horizon * self.N_trainingObj,))
        for i in range(self.N_horizon):
            nominal_values = np.array([self.states_real[3,i]*self.cost[0],self.states_real[4,i]*self.cost[1]]).reshape(self.N_trainingObj,)
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
            state_sim_cur = np.zeros(6,)
            for j in range(self.N_horizon):
                # get initial states received from the simulated robot
                state_sim_cur[:,] = self.states_sim[:, j]
                param_values_NN = self.NN.Auto_to_Neural_Format(sigma_curr.reshape(self.theta.shape[0],))
                self.NN.params_values = param_values_NN
                inpt = self.inpts[:, j]
                # add neural network result
                NN_input = np.array(
                    [(inpt[0] + 2.0) / 10.0, (inpt[1] + 2.0) / 10.0, (state_sim_cur[3] + 2.0) / 10.0, (state_sim_cur[4] + 2.0) / 10.0])

                NN_output, _ = self.NN.full_forward_propagation(np.transpose(NN_input.reshape(1, NN_input.shape[0])))

                # get output from previously trained Real-to-Kin case
                #NN2_output, _ = self.NN2.full_forward_propagation(np.transpose(NN_input.reshape(1, NN_input.shape[0])))

                state_sim_cur[3] = state_sim_cur[3] + NN_output[0] #+ NN2_output[0]
                state_sim_cur[4] = state_sim_cur[4] + NN_output[1] #+ NN2_output[1]

                h_cost = np.array([state_sim_cur[3]*self.cost[0],state_sim_cur[4]*self.cost[1]]).reshape(self.N_trainingObj,)
                self.h_est[(j) * self.N_trainingObj:(j) * self.N_trainingObj + self.N_trainingObj, i] = h_cost.reshape(self.N_trainingObj,)

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

        if self.theta.shape[1] == self.N_horizon*2:
            self.theta = self.theta[:,0].reshape(self.theta.shape[0],1)

        # initialize robot parameters to finalized parameter solution
        #self.NN.params_values = self.NN.Auto_to_Neural_Format(self.theta.reshape(self.theta.shape[0], ))