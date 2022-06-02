from sim.robots.RobotBase import RobotBase
from sim.robots.scalear_leg.scalar_sim import pyb_sim
from sim.type.definitions import *
import time
from scipy import signal

class Pybullet2(RobotBase):
    def __init__(self):
        super().__init__()
        urdf_filename = '/home/alexander/AbstractedRobot/sim/robots/scalear_leg/urdf_scalar_6DoF/urdf/SCALAR_6DoF.urdf'
        self.my_sim = pyb_sim(urdf_filename=urdf_filename, DoFnum=6, delta_t=self.run.DT)
        self.leg = 1
        self.other_leg = 0
        self.run.to_thread=False
        self.inpt = DefDict(POS_3D)
        self.task_space = T_MAT
        self.joint_space = DefDict(joint_pos(6))
        self.other_leg_pos = [0.25,0.0,-0.25]
        self.inpt.data = self.other_leg_pos
        self.joint_space.data = self.ik(self.inpt)
        self.joint_angles_other_leg = self.joint_space.data.as_list()
        self.auto_tuner = None
        self.moving_average_dx = []
        self.moving_average_dy = []
        self.moving_average_total = 100

        self.low_pass_filter_dx = []
        self.low_pass_filter_dy = []
        self.low_pass_filter_horizon = 100
        self.b, self.a = signal.butter(2, 0.01)

    def drive(self, inpts, timestamp):
        #inputs: desired motor angles in rad, order: (Shoulder, q11, q21, wrist1, wrist2, wrist3) * 4 for 4 legs
        self.inpt = inpts
        self.joint_space.data = self.ik(self.inpt)
        joint_angles_leg = self.joint_space.data.as_list()
        joint_angles_legs = [0]*12
        joint_angles_legs[6*self.leg:6*self.leg+6] = joint_angles_leg
        joint_angles_legs[6*self.other_leg:6*self.other_leg+6] = self.joint_angles_other_leg
        self.my_sim.movetoPose(joint_angles_legs)

        return self.state

    def sense(self):
        # Return current motor angles in rad, order: (Shoulder, q11, q21, wrist1, wrist2, wrist3) * 4 for 4 leg
        joint_angles_legs  = self.my_sim.getJointAngles()
        joint_angles_leg = joint_angles_legs[6*self.leg:6*self.leg+6]
        self.outpt.data = joint_angles_leg

        return self.outpt

    def observe_state(self):
        state = self.state.data.as_list()
        self.state.set_data(self.fk(self.outpt))
        self.calc_vel(pre_state=state, curr_state=self.state.data.as_list())

        return self.state

    def clock(self, t):
        #to update scalar
        self.my_sim.setTimestep(self.run.DT)
        self.my_sim.step()
        return t + self.run.DT

    def reset(self):
        self.my_sim.reset()

    def calc_vel(self, pre_state, curr_state):
        dx = (curr_state[0] - pre_state[0]) / self.run.DT
        dy = (curr_state[1] - pre_state[1]) / self.run.DT
        dz = (curr_state[2] - pre_state[2]) / self.run.DT
        inpt = self.inpt.data.as_list()
        # add neural network result
        NN_input = np.array([(inpt[0]+2.0)/10.0,(inpt[1]+2.0)/10.0,(dx+2.0)/10.0,(dy+2.0)/10.0])
        NN_output, _ = self.auto_tuner.NN.full_forward_propagation(np.transpose(NN_input.reshape(1, NN_input.shape[0])))
        dx = dx + NN_output[0][0] + 0.4
        dy = dy + NN_output[1][0] - 0.4
        """
        self.low_pass_filter_dx.append(dx)
        self.low_pass_filter_dy.append(dy)

        if len(self.low_pass_filter_dx) == self.low_pass_filter_horizon:
            del self.low_pass_filter_dx[0]
            self.low_pass_filter_dx.append(dx)
            dx = signal.lfilter(self.b, self.a, self.low_pass_filter_dx)[-1]

        if len(self.low_pass_filter_dy) == self.low_pass_filter_horizon:
            del self.low_pass_filter_dy[0]
            self.low_pass_filter_dy.append(dy)
            dy = signal.lfilter(self.b, self.a, self.low_pass_filter_dy)[-1]

        self.moving_average_dx.append(dx)
        self.moving_average_dy.append(dy)

        if len(self.moving_average_dx) > self.moving_average_total:
            self.moving_average_dx.pop(0)
            self.moving_average_dy.pop(0)
            dx = sum(self.moving_average_dx) / len(self.moving_average_dx)
            dy = sum(self.moving_average_dy) / len(self.moving_average_dy)

        self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}
        """

        self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}

