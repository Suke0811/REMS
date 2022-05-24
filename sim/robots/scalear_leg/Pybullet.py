from sim.robots import RobotBase
from sim.robots.scalear_leg.scalar_sim import pyb_sim


class Pybullet(RobotBase):
    def __init__(self):
        super().__init__()

        urdf_filename = '/home/max/ucla/real2sim/Real-to-Sim/sim/robots/urdf_scalar_6DoF/urdf/SCALAR_6DoF.urdf'
        self.DT = 0.01
        self.my_sim = pyb_sim(urdf_filename=urdf_filename, DoFnum=6, delta_t=self.DT)


    def drive(self, inpts, timestamp):
        #inputs: desired motor angles in rad, order: (Shoulder, q11, q21, wrist1, wrist2, wrist3) * 4 for 4 legs
        self.inpt = inpts
        self.my_sim.movetoPose(inpts)
        return self.state

    
    def sense(self):
        # Return current motor angles in rad, order: (Shoulder, q11, q21, wrist1, wrist2, wrist3) * 4 for 4 legs
        return self.my_sim.getJointAngles()

    def clock(self, t):
        #to update scalar
        self.my_sim.setTimestep(self.DT)
        self.my_sim.step()
        return t + self.DT

    def reset(self):
        self.my_sim.reset()

