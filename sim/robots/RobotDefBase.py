from sim.type import DefDict


class RobotDefBase:
    def __init__(self):
        """init with a specific initial state (optional) """
        self.inpt: DefDict = None
        self.state: DefDict = None
        self.outpt: DefDict = None
        self.info = []
        self.joint_space: DefDict = None
        self.task_space: DefDict = None
        self.jacobian: DefDict = None


    def define(self):
        """Definitions of the robot"""
        raise NotImplementedError

    def control(self, *args):
        """run controller assigned to the robot"""
        raise NotImplementedError

    def fk(self, jointspace: DefDict):
        raise NotImplementedError

    def ik(self, taskspace: DefDict):
        raise NotImplementedError

    def jb(self, jointspace: DefDict):
        raise NotImplementedError
