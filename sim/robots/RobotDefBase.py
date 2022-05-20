from sim.robots.RunConfig import RunConfig
from sim.robots import KinematicsBase
from sim.type.definitions import *
from sim.type import DefDict


class RobotDefBase:
    def __init__(self):
        """init with a specific initial state (optional) """
        self.inpt = None
        self.state = None
        self.outpt = None
        self.kinematics: KinematicsBase = None
        self.info = {}


    def define(self):
        """Definitions of the robot"""
        raise NotImplementedError


    def control(self, *args):
        """run controller assigned to the robot"""
        pass
