from rems.typing import DefDict


class RobotDefBase:
    def __init__(self, *args, **kwargs):
        """init with a specific initial state (optional) """
        self.name = 'robot'
        self.inpt = DefDict()
        self.state = DefDict()
        self.outpt = DefDict()
        self.drive_space = DefDict()
        self.sense_space = DefDict()
        self.info = None
        self.joint_space = DefDict()
        self.task_space = DefDict()
        self.jacobian = DefDict()
        self.defs = dict(inpt=self.inpt, state=self.state, outpt=self.outpt, info=self.info,
                         joint_space=self.joint_space, task_space=self.task_space, jacobian=self.jacobian)
        for name, d in self.defs.items():
            try:
                d.set_name(name)
            except AttributeError:
                pass

    def define(self, drive_space = None, sense_space = None, *args, **kwargs):
        """Definitions of the robot"""
        if drive_space is not None:
            for k, v in drive_space.items():
                try:
                    current = self.drive_space[k]
                except KeyError:
                    self.drive_space.add_def({k:v})
                    continue
                if not isinstance(current, DefDict):
                    current = DefDict(current)
                current.add_def(v)
                self.drive_space[k] = v
        if sense_space is not None:
            for k, v in sense_space.items():
                try:
                    current = self.sense_space[k]
                except KeyError:
                    self.sense_space.add_def({k:v})
                    continue
                if not isinstance(current, DefDict):
                    current = DefDict(current)
                current.add_def(v)
                self.sense_space[k] = v

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

