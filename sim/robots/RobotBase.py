from sim.robots.RunConfig import RunConfig
from sim.bind.BasicDeviceBase import BasicDeviceBase
from sim.robots.RobotDefBase import RobotDefBase


class RobotBase(RobotDefBase, BasicDeviceBase):
    def __init__(self, *args, **kwargs):
        """init with a specific initial stat) """
        super().__init__(*args, **kwargs)
        self.inpt = {}
        self._t_minus_1 = 0.0       # delta t may not be a constant
        self.info = {}
        # Run settings
        self.drivers = []
        self.sensors = []
        self.state_observers = []
        # Run settings
        self.run = RunConfig()
        self.home_position = self.joint_space

    def init(self, *args, **kwargs):
        """Initialization necessary for the robot. call all binded objects' init
        """
        super().init()
        [d.init() and s.init() for d, s in zip(self.drivers, self.sensors)]
        pass

    def reset(self, init_state, t):
        """process necessary to reset the robot without restarting"""
        pass

    def drive(self, inpt, timestamp):
        """drive the robot to the next state
        :param inpts: left, right wheel velocities
        :return full state feedback"""
        self.inpt.data = inpt
        self.joint_space.data = self.inpt
        for d in self.drivers:
            d.drive(self.joint_space)

    def sense(self):
        """generate the sensor reading
        :return output"""
        for s in self.sensors:
            self.outpt.data = s.sense()
        return self.outpt

    def observe_state(self):
        """get current state"""
        return self.state

    def clock(self, t):
        self._t_minus_1 = t
        return t + self.run.DT

    def overwrite_robot(self, state=None, outpt=None, inpt=None):
        if state is not None: self.state = state
        if outpt is not None: self.outpt = outpt
        if inpt is not None: self.inpt = inpt

