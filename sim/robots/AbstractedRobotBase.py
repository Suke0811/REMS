from sim.robots.RunConfig import RunConfig
from sim.robots import KinematicsBase
from sim.type.definitions import *
from sim.type import DefDict
from sim.robots import RobotBase


class AbstractedRobotBase(RobotBase):
    def __init__(self, *args):
        """init with a specific initial state (optional) """
        self.inpt = {}
        self.state = DefDict(POS_2D.update(VEL_POS_2D))
        self.outpt = None
        self._t_minus_1 = 0.0       # delta t may not be a constant
        self.info = {}
        # Run settings
        self.run = RunConfig()

    def define(self):
        """necessity to be a robot (well not necessary tho)"""
        self.kinematics: KinematicsBase = KinematicsBase()
        self.controllers = {}
        self.drivers = {}
        self.sensors = {}
        self.state_observers = {}


    def control(self, *args):
        """run controller assigned to the robot"""
        pass



