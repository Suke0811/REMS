from sim.robots import KinematicsBase


class DifferentialDriveKinematics(KinematicsBase):
    def fk(self, jointspace):
        super().fk(jointspace)
        taskspace = jointspace
        return taskspace

    def ik(self, taskspace):
        jointspace = taskspace
        return jointspace

    def jb(self, jointspace):
        pass

    def constants(self):
        pass
