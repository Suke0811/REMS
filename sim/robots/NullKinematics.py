from sim.robots import KinematicsBase


class NullKinnematics(KinematicsBase):
    def fk(self, jointspace):
        super().fk(jointspace)
        return self.task_definition

    def ik(self, taskspace):
        super().ik(taskspace)
        return self.joint_definition

    def jb(self, jointspace):
        super().jb(jointspace)
        return self.jacobian_definition

    def constants(self):
        pass
