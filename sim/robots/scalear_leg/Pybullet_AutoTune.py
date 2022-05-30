from sim.robots.RobotBase import RobotBase
from sim.robots.scalear_leg.scalar_sim import pyb_sim
from sim.type.definitions import *
import time

class Pybullet2(RobotBase):
    def __init__(self):
        super().__init__()
        urdf_filename = '/home/alexander/AbstractedRobot/sim/robots/scalear_leg/urdf_scalar_6DoF/urdf/SCALAR_6DoF.urdf'
        self.my_sim = pyb_sim(urdf_filename=urdf_filename, DoFnum=6, delta_t=self.run.DT)
        self.auto_tuner = None
        self.leg = 1
        self.run.to_thread=False

    def drive(self, inpts, timestamp):
        #inputs: desired motor angles in rad, order: (Shoulder, q11, q21, wrist1, wrist2, wrist3) * 4 for 4 legs
        self.inpt = inpts
        self.joint_space.data = self.ik(self.inpt)
        joint_angles_leg = self.joint_space.data.as_list()
        joint_angles_legs = [0]*12
        joint_angles_legs[6*self.leg:6*self.leg+6] = joint_angles_leg
        self.my_sim.movetoPose(joint_angles_legs)

        return self.state

    def sense(self):
        # Return current motor angles in rad, order: (Shoulder, q11, q21, wrist1, wrist2, wrist3) * 4 for 4 leg
        joint_angles_legs  = self.my_sim.getJointAngles()
        joint_angles_leg = joint_angles_legs[6*self.leg:6*self.leg+6]
        self.outpt.data = joint_angles_leg

        return self.outpt

    def observe_state(self):
        state = self.state
        self.state.set_data(self.fk(self.outpt))
        self.calc_vel(pre_state=state, curr_state=self.state)

        return self.state

    def clock(self, t):
        #to update scalar
        self.my_sim.setTimestep(self.run.DT)
        self.my_sim.step()
        return t + self.run.DT

    def reset(self):
        self.my_sim.reset()

    def calc_vel(self, pre_state, curr_state):
        prev_state = pre_state.data_as(POS_3D).data.as_list()
        self.state.data = self.task_space
        next_state = curr_state.data_as(POS_3D).data.as_list()
        dx = (next_state[0] - prev_state[0]) / self.run.DT
        dy = (next_state[1] - prev_state[1]) / self.run.DT
        dz = (next_state[2] - prev_state[2]) / self.run.DT
        inpt = self.inpt.data.as_list()
        # add neural network result
        NN_input = np.array([(inpt[0]+2.0)/10.0,(inpt[1]+2.0)/10.0,(dx+2.0)/10.0,(dy+2.0)/10.0])
        NN_output, _ = self.auto_tuner.NN.full_forward_propagation(np.transpose(NN_input.reshape(1, NN_input.shape[0])))
        dx = dx + NN_output[0] + 1
        dy = dy + NN_output[1] - 1

        self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}

