from sim.robots.RobotBase import RobotBase


class NopRobot(RobotBase):
    def __init__(self):
        super().__init__()
        self.run.name = 'Nop'
        self.run.realtime = False
        self.run.to_thread = True

    def drive(self, inpt, timestamp):
        pass

    def sense(self):
        return self.outpt

    def observe_state(self):
        return self.state

