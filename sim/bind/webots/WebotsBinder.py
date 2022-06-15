from sim.robots import RobotBase
from sim.bind.webots.WebotsDrive import WebotsDrive
from sim.bind.webots.WebotsSense import WebotsSense
from sim.typing.definitions import *
from controller import Supervisor


class WebotsBinder(RobotBase):
    """You will receive updates in the next lab"""
    def __init__(self, robot_name):
        super().__init__()
        # gets robot instance from the webots
        self.run.name = robot_name
        self._robot = Supervisor()
        #get robot node, and translation and rotation fields
        self._robot_node = self._robot.getFromDef(robot_name)
        self._trans_field = self._robot_node.getField("translation")
        self._rotation_field = self._robot_node.getField("rotation")
        # get the time step of the current world.
        self._timestep = int(self._robot.getBasicTimeStep())
        self.run.DT = self._timestep / 1000
        # add drivers and sensors
        self.drivers.append(WebotsDrive(self._robot, self.inpt))
        self.sensors.append(WebotsSense(self._robot, self._timestep, self.outpt))

    def init(self):
        # init motors and sensors
        super().init()
        self._robot.step(self._timestep)

    def reset(self, state):
        rot = DefDict(ROT_2D)
        self._trans_field.setSFVec3f(DefDict(POS_3D).format(state).list())  # move robot to the init state
        self._rotation_field.setSFRotation([0, 1, 0, rot])
        self._robot_node.setVelocity(DefDict((VEL_POS_3D, VEL_ROT_3D)).format(state).list())
        self._robot_node.resetPhysics()  # reset physics

    def drive(self, inpts, timestamp):
        """drive the robot to the next state
        :param inpts: left, right wheel velocities
        :return full state feedback"""
        self.inpt.set(inpts)
        self.drivers.drive(inpts)

    def observe_state(self):
        self.state.set(self.wb_sense.state)
        return self.state

    def sense(self):
        """generate the sensor reading"""
        self.outpt.set(self.wb_sense.sense())
        return self.outpt
