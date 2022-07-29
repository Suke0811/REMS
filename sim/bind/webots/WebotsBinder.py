import numpy as np

from sim.robots import RobotBase
from sim.bind.webots.WebotsDrive import WebotsDrive
from sim.bind.webots.WebotsSense import WebotsSense
from sim.typing.definitions import *
from controller import Supervisor, Robot
from scipy.spatial.transform import Rotation as R
import os




WEBOTS_POS = dict(x=float, y=float, z=float)
WEBOTS_ROT = dict(th_x=float, th_y=float, th_z=float)
WEBOTS_POS_VEL = dict(d_x=float, d_y=float, d_z=float)
WEBOTS_ROT_VEL = dict(d_th_x=float, d_th_y=float, d_th_z=float)

class WebotsBinder(RobotBase):
    """You will receive updates in the next lab"""
    def __init__(self):
        super().__init__()
        self.wb_state = DefDict((WEBOTS_POS, WEBOTS_ROT, WEBOTS_POS_VEL, WEBOTS_ROT_VEL))
        # gets robot instance from the webots
        #get robot node, and translation and rotation fields


    def init(self):
        # init motors and sensors
        os.environ['WEBOTS_ROBOT_NAME'] = self.run.name
        self._robot = Supervisor()
        self._robot_node = self._robot.getFromDef(self.run.name)
        print(self._robot.getName())
        if self._robot_node is None:
            self._robot_node = self._robot.getSelf()
            if self._robot_node is None:
                raise ImportError(f'the Webots robot {self.run.name} could not found')
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
        self.wb_state.set(state)
        self._trans_field.setSFVec3f(self.wb_state.filter(WEBOTS_POS).list())  # move robot to the init state
        self._rotation_field.setSFRotation([0, 1, 0, self.wb_state.get('th_z')])
        self._robot_node.setVelocity(self.wb_state.filter((WEBOTS_POS_VEL, WEBOTS_ROT_VEL)).list())
        self._robot_node.resetPhysics()  # reset physics
        self.clock(0)

    def drive(self, inpts, timestamp):
        """drive the robot to the next state
        :param inpts: left, right wheel velocities
        :return full state feedback"""
        self.inpt.set(inpts)
        self.wb_driver.drive(inpts, timestamp)

    def observe_state(self):
        """Collect ground truth position, orientation, and velocity"""
        pos = self._robot_node.getPosition()  # Position, 1 by 3
        r_vec = self._robot_node.getOrientation()  # this is rotation matrix in 1 by 9 vector
        r = R.from_matrix([r_vec[i:i + 3] for i in range(0, len(r_vec), 3)])
        euler = r.as_euler('xyz')  # change R to euler
        vel = self._robot_node.getVelocity()  # get velocity, 1 by 6
        state = np.concatenate([pos, euler, vel])
        self.state.set(state)
        return self.state

    def sense(self):
        """generate the sensor reading"""
        self.outpt.set(self.wb_sensor.sense())
        return self.outpt

    def clock(self, t):
        """to update Webots"""
        self._robot.step(self._timestep)
        return t + self._timestep / 1000

