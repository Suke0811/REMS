from sim.robots.KinematicsBase import KinematicsBase
from sim.type import DefDict
from sim.type.definitions import *


class ScalerManipulatorKinematics(KinematicsBase):
    NUM_JOINTS = 6

    def __init__(self):
        super().__init__()
        # FK, IK uses joint pos
        self.joint_definition = DefDict(joint_pos(self.NUM_JOINTS))
        # Task space is 3D position and quaternion
        self.task_definition = DefDict(POS_3D, QUAT)
        # Jacobian is not available
        self.jacobian_definition = None

    def fk(self, jointspace: DefDict):
        pass

    def ik(self, taskspace: DefDict):
        pass

    def constants(self):
        pass
