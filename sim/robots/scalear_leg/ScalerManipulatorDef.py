from sim.robots.RobotDefBase import RobotDefBase
from sim.type import DefDict
from sim.type.definitions import POS_2D, joint_pos, joint_vel
from sim.type import DefDict
from sim.type.definitions import *
from sim.robots.scalear_leg.kinematics.SCALAR_kinematics import ScalerKinematics

from sim.robots.RobotBase import RobotBase

class ScalerManipulator(RobotDefBase):
    NUM_JOINTS = 6
    WHICH_LEG = 3

    def __init__(self):
        RobotDefBase.__init__(self)
        self.kin = ScalerKinematics()


    def define(self):
        # input is 2D pos
        self.inpt = DefDict(POS_2D)
        # state is joint pos and vel
        self.state = DefDict(joint_pos(self.NUM_JOINTS), joint_vel(self.NUM_JOINTS))
        # sensors are joins pos and vel
        self.outpt = DefDict(joint_pos(self.NUM_JOINTS), joint_vel(self.NUM_JOINTS))

        self.joint_definition = DefDict(joint_pos(self.NUM_JOINTS))
        # Task space is 3D position and quaternion
        self.task_definition = T_MAT
        self.task_definition.data = dict(r11=1.0, r22=-1.0, r33=-1.0)
        # Jacobian is not available
        self.jacobian_definition = None

    def fk(self, jointspace: DefDict):
        self.joint_space.data = jointspace
        return self.task_space.set_data(self.kin.scalar_forward_kinematics(which_leg=self.WHICH_LEG, joint_angles=self.joint_space.data.as_list()))

    def ik(self, taskspace: DefDict):
        self.task_space = taskspace
        return self.joint_space.set_data(self.kin.scalar_inverse_kinematics(which_leg=self.WHICH_LEG, T_shi_wrist3=self.task_space.data.as_square_numpy()))

    def observe_state(self):
        self.state = self.fk(self.outpt)
        return self.state
