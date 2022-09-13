import numpy as np
from sim.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType

from sim.robots import RobotBase
from sim.device.webots.WebotsDrive import WebotsDrive
from sim.device.webots.WebotsSense import WebotsSense
from sim.typing.definitions import *
from controller import Supervisor, Robot
from scipy.spatial.transform import Rotation as R
import os

WEBOTS_POS = dict(x=Pos, y=Pos, z=Pos)
WEBOTS_ROT = dict(th_x=Ang, th_y=Ang, th_z=Ang)
WEBOTS_POS_VEL = dict(d_x=Vel, d_y=Vel, d_z=Vel)
WEBOTS_ROT_VEL = dict(d_th_x=AngVel, d_th_y=AngVel, d_th_z=AngVel)

WEBOTS_MOTOR = dict()

class WebotsBinder(RobotBase):
    """You will receive updates in the next lab"""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.wb_state = DefDict((WEBOTS_POS, WEBOTS_ROT, WEBOTS_POS_VEL, WEBOTS_ROT_VEL))
        self.run.name = 'Webots'
        # gets robot instance from the webots
        #get robot node, and translation and rotation fields

    def init(self, *args, **kwargs):
        # init motors and sensors
        os.environ['WEBOTS_ROBOT_NAME'] = self.name
        self._robot = Supervisor()
        self._robot_node = self._robot.getFromDef(self.name)
        if self._robot_node is None:
            self._robot_node = self._robot.getSelf()
            if self._robot_node is None:
                raise ImportError(f'the Webots robot {self.name} could not found')
        self._trans_field = self._robot_node.getField("translation")
        self._rotation_field = self._robot_node.getField("rotation")
        # get the time step of the current world.
        self._timestep = int(self._robot.getBasicTimeStep())
        self.run.DT = self._timestep / 1000

        self.add_device(WebotsDrive(self._robot, ))
        self.add_device(WebotsSense(self._robot, self._timestep))
        super().init()
        self._robot.step(self._timestep)

    def reset(self, state, t):
        if state is not None:
            self.wb_state.set(state)
            self._trans_field.setSFVec3f(self.wb_state.filter(WEBOTS_POS).list())  # move robot to the init state
            self._rotation_field.setSFRotation([0, 1, 0, self.wb_state.get('th_z')])
            self._robot_node.setVelocity(self.wb_state.filter((WEBOTS_POS_VEL, WEBOTS_ROT_VEL)).list())
        self._robot_node.resetPhysics()  # reset physics
        self.clock(0)

    def observe_state(self):
        """Collect ground truth position, orientation, and velocity"""
        pos = self._robot_node.getPosition()  # Position, 1 by 3
        r_vec = self._robot_node.getOrientation()  # this is rotation matrix in 1 by 9 vector
        r = R.from_matrix([r_vec[i:i + 3] for i in range(0, len(r_vec), 3)])
        euler = r.as_euler('xyz')  # change R to euler
        vel = self._robot_node.getVelocity()  # get velocity, 1 by 6
        state = np.concatenate([pos, euler, vel])
        self.wb_state.set(state)
        self.state.set(self.wb_state)
        return self.state


    def clock(self, t):
        """to update Webots"""
        self._robot.step(self._timestep)
        return t + self._timestep / 1000


    @classmethod
    def drive_space_def(cls, driver_names, *args, **kwargs):
        return DefDict({device.device_name: device.create_drive_space(driver_names) for device in cls.DEVICE_LIST})

    @classmethod
    def sense_space_def(cls, sensor_names, *args, **kwargs):
        return DefDict({device.device_name: device.create_drive_space(sensor_names) for device in cls.DEVICE_LIST})

