from sim.robots.RunConfig import RunConfig
from sim.robots.bind import BasicDeviceBase
from sim.type.definitions import *
from sim.type import DefDict

class RobotBaseBasic(BasicDeviceBase):
    def __init__(self, run_config=RunConfig()):
        """init with a specific initial state (optional) """
        self.inpt = {}
        self._t_minus_1 = 0.0       # delta t may not be a constant
        self.info = {}
        # Run settings
        self.run = run_config
        self.drivers = []
        self.sensers = []
        self.state_observers = []
        self.state = None
        self.outpt = None
        self.inpt = None

    def init(self, init_state=None):
        """Initialization necessary for the robot. call all binded objects' init
        """
        super().init()
        [d.init() and s.init() for d, s in zip(self.drivers, self.sensers)]
        pass

    def reset(self, **kwargs):
        """process necessary to reset the robot without restarting"""
        pass

    def drive(self, inpt, timestamp):
        """drive the robot to the next state
        :param inpts: left, right wheel velocities
        :return full state feedback"""
        self.inpt = inpt

    def sense(self, definition=None):
        """generate the sensor reading
        :return output"""
        return self.outpt

    def observe_state(self, definition=None):
        """get current state"""
        return self.state

    def clock(self, t):
        self._t_minus_1 = t
        return t + self.run.DT

    def overwrite_robot(self, state=None, outpt=None, inpt=None):
        if state is not None: self.state = state
        if outpt is not None: self.outpt = outpt
        if inpt is not None: self.inpt = inpt

