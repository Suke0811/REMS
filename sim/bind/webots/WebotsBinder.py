from sim.robots import RobotBase
from sim.bind.webots.WebotsDrive import WebotsDrive
from sim.bind.webots.WebotsSense import WebotsSense
from sim.typing.definitions import *
from controller import Supervisor


class WebotsBinder(RobotBase):
    """You will receive updates in the next lab"""
    def __init__(self):
        super().__init__()
        # gets robot instance from the webots
        #get robot node, and translation and rotation fields


    def init(self):
        # init motors and sensors
        self._robot = Supervisor()
        self._robot_node = self._robot.getFromDef(self.run.name)
        self._trans_field = self._robot_node.getField("translation")
        self._rotation_field = self._robot_node.getField("rotation")
        # get the time step of the current world.
        self._timestep = int(self._robot.getBasicTimeStep())
        self.run.DT = self._timestep / 1000
        # add drivers and sensors
        self.wb_driver = WebotsDrive(self._robot, self.joint_space)
        self.wb_sensor = WebotsSense(self._robot, self._timestep, self.outpt)
        self.wb_driver.init()
        self.wb_driver.open()
        self.wb_sensor.init()
        self.wb_sensor.open()

        self._robot.step(self._timestep)

    def reset(self, state, t):
        rot = DefDict(ROT_2D)
        self._trans_field.setSFVec3f(DefDict(POS_3D).format(state).list())  # move robot to the init state
        self._rotation_field.setSFRotation([0, 1, 0, rot.get()])
        self._robot_node.setVelocity(DefDict((VEL_POS_3D, VEL_ROT_3D)).format(state).list())
        self._robot_node.resetPhysics()  # reset physics
        self.clock(0)

    def drive(self, inpts, timestamp):
        """drive the robot to the next state
        :param inpts: left, right wheel velocities
        :return full state feedback"""
        self.inpt.set(inpts)
        self.wb_driver.drive(inpts, timestamp)

    def observe_state(self):
        #self.state.set(self.wb_sensor.state)
        return self.state

    def sense(self):
        """generate the sensor reading"""
        self.outpt.set(self.wb_sensor.sense())
        return self.outpt

    def clock(self, t):
        """to update Webots"""
        self._robot.step(self._timestep)
        return t + self._timestep / 1000

