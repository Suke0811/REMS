from sim.robots.RobotDefBase import RobotDefBase
from sim.robots.scalear_leg.ScalerManipulatorKinematics import ScalerManipulatorKinematics
from sim.type import DefDict
from sim.type.definitions import POS_2D, joint_pos, joint_vel

class ScalerManipulator(RobotDefBase):
    def __init__(self):
        super().__init__()

    def define(self):
        self.kinematics = ScalerManipulatorKinematics()
        num_joints = self.kinematics.NUM_JOINTS
        # input is 2D pos
        self.inpt = DefDict(POS_2D)
        # state is joint pos and vel
        self.state = DefDict(joint_pos(num_joints), joint_vel(num_joints))
        # sensors are joins pos and vel
        self.outpt = DefDict(DefDict(joint_pos(num_joints), joint_vel(num_joints)))

