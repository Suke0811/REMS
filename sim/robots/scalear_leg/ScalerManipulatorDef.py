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
        self.inpt = DefDict(POS_3D)
        # state is joint pos and vel
        self.state = DefDict(POS_3D, VEL_POS_3D)
        # sensors are joins pos and vel
        self.outpt = DefDict(joint_pos(self.NUM_JOINTS), joint_vel(self.NUM_JOINTS))

        self.joint_space = DefDict(joint_pos(self.NUM_JOINTS))
        # Task space is 3D position and quaternion
        self.task_space = T_MAT
        # Jacobian is not available
        self.jacobian = None
        self.info = DefDict({'h2_norm':0, 'h2_norm_x':0, 'h2_norm_y':0})

    def fk(self, jointspace: DefDict):
        d = self.kin.scalar_forward_kinematics(which_leg=self.WHICH_LEG,
                                               joint_angles=self.joint_space.format_data(jointspace).data.as_list())

        return self.task_space.format_data( Tmat2dict_rule.bind(d))

    def ik(self, taskspace: DefDict):
        self.task_space.data = dict(r11=1, r22=-1, r33=-1,
                                    r12=0, r13=0, r21=0, r23=0, r31=0, r32=0)
        return self.joint_space.format_data(
            self.kin.scalar_inverse_kinematics(which_leg=self.WHICH_LEG,
                                               T_shi_wrist3=self.task_space.format_data(taskspace).ruled_get(),
                                               prev_angles=self.joint_space.data.as_list()))


