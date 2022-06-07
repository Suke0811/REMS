from sim.robots.RobotBase import RobotBase
from sim.robots.scalear_leg.scalar_sim import pyb_sim
from sim.type.definitions import *
import time

urdf_filename = '/home/yusuke/PycharmProjects/pySiLVIA_lib/AbstractedRobot/sim/robots/scalear_leg/urdf_scalar_6DoF/urdf/SCALAR_6DoF.urdf'


class Pybullet(RobotBase):
    def __init__(self):
        super().__init__()
        self.leg = 0
        self.other_leg = 1


    def init(self, *args):
        self.my_sim = pyb_sim(urdf_filename=urdf_filename, DoFnum=6, delta_t=self.run.DT)
        self.other_leg_pos = [0.25, 0.0, -0.25]
        self.inpt.data = self.other_leg_pos
        self.joint_space.data = self.ik(self.inpt)
        self.joint_angles_other_leg = self.joint_space.data.list()


    def drive(self, inpts, timestamp):
        #inputs: desired motor angles in rad, order: (Shoulder, q11, q21, wrist1, wrist2, wrist3) * 4 for 4 legs
        self.inpt = inpts
        self.joint_space.data = self.ik(self.inpt)
        joint_angles_leg = self.joint_space.data.list()
        joint_angles_legs = [0]*12
        joint_angles_legs[6*self.leg:6*self.leg+6] = joint_angles_leg
        joint_angles_legs[6*self.other_leg:6*self.other_leg+6] = joint_angles_leg
        self.my_sim.movetoPose(joint_angles_legs)

        return self.state

    def sense(self):
        # Return current motor angles in rad, order: (Shoulder, q11, q21, wrist1, wrist2, wrist3) * 4 for 4 leg
        joint_angles_legs = self.my_sim.getJointAngles()
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

    def reset(self, *args):
        self.my_sim.reset()

    def calc_vel(self, pre_state, curr_state):
        prev_state = pre_state.data_as(POS_3D).data.list()
        next_state = curr_state.data_as(POS_3D).data.list()
        dx = (next_state[0] - prev_state[0]) / self.run.DT
        dy = (next_state[1] - prev_state[1]) / self.run.DT
        dz = (next_state[2] - prev_state[2]) / self.run.DT
        self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}

