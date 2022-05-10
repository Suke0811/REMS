from sim.robots import AbstractedRobotBase
from sim.robots.differential_drive import DifferentialDriveKinematics
from sim.type.definitions import *
from sim.type import DefDict


class TwoWheeledRobot(AbstractedRobotBase):
    def define(self):
        super().define()
        """necessity to be a robot (well not necessary tho)"""
        self.state = DefDict(STATE_2D.update(STATE_VEL_2D))
        self.outpt = dict(lidar_f=float(), lidar_r=float(), mag_x=float(), mag_y=float(), gyro_z=float())
        self.kinematics = DifferentialDriveKinematics()
        self.controllers = {}
        self.drivers = {}
        self.sensors = {}
        self.state_observers = {}

    def init(self):
        """Initialization necessary for the robot
        the use of __init__ should be minimized"""
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
