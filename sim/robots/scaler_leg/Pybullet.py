from sim.robots.RobotBase import RobotBase
from sim.robots.scaler_leg.scalar_sim import pyb_sim
from sim.typing.definitions import *
from sim.utils.urdf_relative_path import urdf_filepath_resolver
from pathlib import Path

URDF_PATH= 'sim/robots/scaler_leg/urdf_scalar_6DoF/urdf/SCALAR_6DoF.urdf'
MESH_DIR = 'sim/robots/scaler_leg/urdf_scalar_6DoF/meshes/'

class Pybullet(RobotBase):
    def __init__(self):
        super().__init__()
        self.leg = 0
        self.other_leg = 1
        self.run.DT = 0.01


    def init(self, *args):
        full_path = Path.cwd().joinpath(URDF_PATH)
        urdf_filename = urdf_filepath_resolver(full_path, MESH_DIR)
        self.my_sim = pyb_sim(urdf_filename=urdf_filename, DoFnum=6, delta_t=self.run.DT)
        self.other_leg_pos = [0.25, 0.0, -0.25]
        self.inpt.set(self.other_leg_pos)
        self.joint_space.set(self.ik(self.inpt))
        self.joint_angles_other_leg = self.joint_space.list()


    def drive(self, inpts, timestamp):
        #inputs: desired motor angles in rad, order: (Shoulder, q11, q21, wrist1, wrist2, wrist3) * 4 for 4 legs
        self.inpt.set(inpts)
        self.joint_space.set(self.ik(self.inpt))
        joint_angles_leg = self.joint_space.list()
        joint_angles_legs = [0]*12
        joint_angles_legs[6*self.leg:6*self.leg+6] = joint_angles_leg
        joint_angles_legs[6*self.other_leg:6*self.other_leg+6] = joint_angles_leg
        self.my_sim.movetoPose(joint_angles_legs)

    def sense(self):
        # Return current motor angles in rad, order: (Shoulder, q11, q21, wrist1, wrist2, wrist3) * 4 for 4 leg
        joint_angles_legs = self.my_sim.getJointAngles()
        joint_angles_leg = joint_angles_legs[6*self.leg:6*self.leg+6]
        self.outpt.set(joint_angles_leg)

        return self.outpt

    def observe_state(self):
        state = self.state.list()
        self.state.set(self.fk(self.outpt))
        self.calc_vel(pre_state=state, curr_state=self.state.list())
        return self.state

    def clock(self, t):
        #to update scalar
        self.my_sim.setTimestep(self.run.DT)
        self.my_sim.step()
        return t + self.run.DT

    def reset(self, *args):
        self.my_sim.reset()

