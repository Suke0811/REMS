from sim.modules import DriveBase, SenseBase, RobotSystem

from sim.constants import DATA
from sim.formulation import INPUT_SPACE, OUTPUT_SPACE, STATE_SPACE

from controller import Robot, Motor, Supervisor
import numpy as np
from scipy.spatial.transform import Rotation as R


class WebotsModel(RobotSystem):
    """You will receive updates in the next lab"""
    def __init__(self, velocity_control=True):
        super().__init__()
        self._init_webots()
        self.run.DT = self._timestep / 1000
        self.run.realtime = True
        self.velocity_control = velocity_control

    def drive(self, inpts, timestamp):
        """drive the robot to the next state
        :param inpts: left, right wheel velocities
        :return full state feedback"""
        self.inpt = inpts
        self._input_to_webots(inpts)
        return self.state

    def sense(self):
        """generate the sensor reading"""
        self._output_from_webots()
        return self.outpt


    def _init_webots(self):
        self._robot = Supervisor()
        self._robot_node = self._robot.getFromDef(DATA.WOODBOT)
        self._trans_field = self._robot_node.getField("translation")
        self._rotation_field = self._robot_node.getField("rotation")
        self._trans_field.setSFVec3f(self.state[0:2] + [0.0])    #move robot to the init state
        self._rotation_field.setSFRotation([0, 1, 0, self.state[2]])
        self._robot_node.setVelocity([0,0,0,0,0,0])
        self._robot_node.resetPhysics()             #reset physics
        # get the time step of the current world.
        self._timestep = int(self._robot.getBasicTimeStep())
        # init motors and sensors
        self._init_motors()
        self._init_sensors()

        self._robot.step(self._timestep)


    def _init_motors(self):
        self._motors = {}
        # motor instances
        for MOTOR in INPUT_SPACE.keys():
            self._motors[MOTOR] = self._robot.getDevice(MOTOR)
        # set motor to velocity control mode
        if self.velocity_control:
            for motor in self._motors.values():
                motor.setPosition(float('inf'))

    def _init_sensors(self):
        self._sensors = {}
        # sensor instances
        for SENSOR in OUTPUT_SPACE.keys():
            self._sensors[SENSOR] = self._robot.getDevice(SENSOR)
        # enable sensors
        for sensor in self._sensors.values():
            sensor.enable(self._timestep)

    def _input_to_webots(self, input):
        # send rotational velocity to each motor
        for key, motor in self._motors.items():
            index = list(self._motors.keys()).index(key)
            if self.velocity_control:
                motor.setVelocity(input[index])
            else:
                motor.setPosition(input[index])
        return self._state_from_webots()

    def _state_from_webots(self):
        """Collect ground truth position, orientation, and velocity"""
        pos = self._robot_node.getPosition() # Position, 1 by 3
        r_vec = self._robot_node.getOrientation()   # this is rotation matrix in 1 by 9 vector
        r = R.from_matrix([r_vec[i:i+3] for i in range(0, len(r_vec), 3)])
        euler = r.as_euler('xyz')       # change R to euler
        vel = self._robot_node.getVelocity()    # get velocity, 1 by 6
        state = pos[0:2] + [(euler[2]), vel[5]] #ee
        self.state = [state[i] for i in range(len(STATE_SPACE))]


    def _output_from_webots(self):
        """Collect sensor data and update self.output"""
        for key, sensor in self._sensors.items():
            index = list(self._sensors.keys()).index(key)
            try: # Webots has two different functions of getValue()
                self.outpt[index] = self._filter_nan(sensor.getValue())
            except AttributeError: # then try getValues()
                self.outpt[index] = self._filter_nan(sensor.getValues())



    def clock(self, t):
        """to update Webots. Webots is synchronous and has its own time step"""
        self._robot.step(self._timestep)
        return t + self._timestep / 1000
