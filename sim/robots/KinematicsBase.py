from sim.type import DefDict
from sim.type.definitions import *

class KinematicsBase:
    def __init__(self):
        self.joint_definition = None
        self.task_definition = None
        self.jacobian_definition = None
        self.constants()

    def fk(self, jointspace: DefDict):
        raise NotImplementedError

    def ik(self, taskspace: DefDict):
        raise NotImplementedError

    def jb(self, jointspace: DefDict):
        raise NotImplementedError

    def constants(self):
        pass
