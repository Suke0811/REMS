from sim.type import DefDict
from sim.type.definitions import *

class KinematicsBase:
    def __init__(self, j_def, t_def=STATE_2D, jac_def=JACOB_3D):
        self.joint_definition = DefDict(j_def)
        self.task_definition = DefDict(t_def)
        self.jacobian_definition = DefDict(jac_def)
        self.constants()

    def fk(self, jointspace: DefDict):
        self.joint_definition.assert_data(jointspace)

    def ik(self, taskspace: DefDict):
        self.task_definition.assert_data(taskspace)

    def jb(self, jointspace: DefDict):
        self.joint_definition.assert_data(jointspace)

    def constants(self):
        pass
