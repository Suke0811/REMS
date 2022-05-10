from sim.modules import RunConfig
from sim.robots import KinematicsBase
from sim.type.definitions import *
from sim.type import DefDict

class AbstractedRobotBase:
    def __init__(self, bind_rule):
        """init with a specific initial state (optional) """
        self.inpt = {}
        self._t_minus_1 = 0.0       # delta t may not be a constant
        self.info = {}
        # Run settings
        self.run = RunConfig()
        self.bind_rule = bind_rule

    def define(self):
        """necessity to be a robot (well not necessary tho)"""
        self.state = DefDict(STATE_2D.update(STATE_VEL_2D))
        self.outpt = dict(lidar_f=float, lidar_r=float, mag_x=float, mag_y=float, gyro_z=float)
        self.kinematics: KinematicsBase = KinematicsBase()
        self.controllers = {}
        self.drivers = {}
        self.sensors = {}
        self.state_observers = {}

    def init(self):
        """Initialization necessary for the robot
        """
        pass

    def reset(self, **kwargs):
        """process necessary to reset the robot without restarting"""
        pass

    def control(self, **kwargs):
        """run controller assigned to the robot"""
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

    def bind(self, controllers=None, drivers=None, sensors=None, state_observers=None):
        if controllers is not None:
            self.controllers.update(controllers)
        if drivers is not None:
            self.drivers.update(drivers)
        if sensors is not None:
            self.sensors.update(sensors)
        if state_observers is not None:
            self.state_observers.update(state_observers)
