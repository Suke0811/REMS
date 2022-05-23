from sim.robots.RobotBase import RobotBase


class KinematicModel(RobotBase):
    def drive(self, inpt, timestamp):
        #
        self.inpt = inpt
        self.joint_space.data = inpt
        self.outpt = self.joint_space
        self.task_space.data = self.fk(self.joint_space)
        self.state = self.task_space

    def sense(self):
        return self.outpt

    def observe_state(self):
        return self.state
