from sim.typing import DefDict


class RobotDefBase:
    def __init__(self):
        """init with a specific initial state (optional) """
        self.inpt: DefDict = None
        self.state: DefDict = None
        self.outpt: DefDict = None
        self.info = None
        self.joint_space: DefDict = None
        self.task_space: DefDict = None
        self.jacobian: DefDict = None
        self.all_defs = [self.inpt, self.state, self.outpt, self.info, self.joint_space, self.task_space, self.jacobian]
        self.all_names = ['inpt', 'state', 'outpt', 'info', 'joint_space', 'task_space', 'jacobian']


    def define(self, *args, **kwargs):
        """Definitions of the robot"""
        for d, name in zip(self.all_defs, self.all_names):
            try:
                d.add_name(name)
            except AttributeError:
                pass

    def fk(self, jointspace: DefDict, *args, **kwargs):
        raise NotImplementedError

    def ik(self, taskspace: DefDict, *args, **kwargs):
        raise NotImplementedError

    def jb(self, jointspace: DefDict, *args, **kwargs):
        raise NotImplementedError

    def inv_jb(self, theta: DefDict, *args, **kwargs):
        raise NotImplementedError

    def fdyn(self, jointspace: DefDict, *args, **kwargs):
        raise NotImplementedError

    def idyn(self, taskspace: DefDict, *args, **kwargs):
        raise NotImplementedError
