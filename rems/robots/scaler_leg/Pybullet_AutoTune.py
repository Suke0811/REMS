from rems.robots.RobotBase import RobotBase
from rems.robots.scaler_leg.scalar_sim import pyb_sim
from rems.typing.definitions import *
from rems.robots.scaler_leg.Pybullet import Pybullet as Base
import time

class Pybullet(Base):
    def __init__(self):
        super().__init__()
        self.auto_tuner = None
        self.leg = 1

        self.moving_average_dx = []
        self.moving_average_dy = []
        self.moving_average_total = 100

        self.moving_average_out1 = []
        self.moving_average_out2 = []
        self.moving_average_total_NN = 100


    def observe_state(self):
        state = self.state
        self.state.set(self.fk(self.outpt))
        self.calc_vel(pre_state=state, curr_state=self.state)

        # filter
        cur_vel = self.state.data_as(VEL_POS_3D).data.list()
        self.moving_average_dx.append(cur_vel[0])
        self.moving_average_dy.append(cur_vel[1])

        if len(self.moving_average_dx) > self.moving_average_total:
            self.moving_average_dx.pop(0)
            self.moving_average_dy.pop(0)
            cur_vel[0] = sum(self.moving_average_dx) / len(self.moving_average_dx)
            cur_vel[1] = sum(self.moving_average_dy) / len(self.moving_average_dy)

        # add neural network result
        NN_input = np.array([cur_vel[0], cur_vel[1]])
        NN_output, _ = self.auto_tuner.NN.full_forward_propagation(np.transpose(NN_input.reshape(1, NN_input.shape[0])))
        cur_vel[0] = cur_vel[0] + NN_output[0]
        cur_vel[1] = cur_vel[1] + NN_output[1]

        self.state.data = {'d_x': cur_vel[0], 'd_y': cur_vel[1], 'd_z': cur_vel[2]}

        return self.state

    def clock(self, t):
        #to update scalar
        self.my_sim.setTimestep(self.run.DT)
        self.my_sim.step()
        return t + self.run.DT

    def reset(self):
        self.my_sim.reset()

    def calc_vel(self, pre_state, curr_state):
        prev_state = pre_state.data_as(POS_3D).data.list()
        self.state.data = self.task_space
        next_state = curr_state.data_as(POS_3D).data.list()
        dx = (next_state[0] - prev_state[0]) / self.run.DT
        dy = (next_state[1] - prev_state[1]) / self.run.DT
        dz = (next_state[2] - prev_state[2]) / self.run.DT
        self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}

