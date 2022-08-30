from sim.typing import DefDict


class RobotDefBase:
    def __init__(self, *args, **kwargs):
        """init with a specific initial state (optional) """
        self.name = 'robot'
        self.inpt: DefDict = None
        self.state: DefDict = None
        self.outpt: DefDict = None
        self.drive_space: DefDict = None
        self.sense_space: DefDict = None
        self.info = None
        self.joint_space: DefDict = None
        self.task_space: DefDict = None
        self.jacobian: DefDict = None
        self.defs = dict(inpt=self.inpt, state=self.state, outpt=self.outpt, info=self.info,
                         joint_space=self.joint_space, task_space=self.task_space, jacobian=self.jacobian)

    def define(self, *args, **kwargs):
        """Definitions of the robot"""
        self.defs = dict(inpt=self.inpt, state=self.state, outpt=self.outpt, info=self.info,
                         joint_space=self.joint_space, task_space=self.task_space, jacobian=self.jacobian)
        for name, d in self.defs.items():
            try:
                d.set_name(name)
            except AttributeError:
                pass

    def fk(self, jointspace: DefDict, *args, **kwargs):
        raise NotImplementedError

    def ik(self, taskspace: DefDict, *args, **kwargs):
        raise NotImplementedError

    def jb(self, jointspace: DefDict, *args, **kwargs):
        raise NotImplementedError

