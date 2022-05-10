from sim.robots import RobotBase
from sim.robots.bind.Webots import WebotsDrive, WebotsSense
from controller import Robot, Motor, Supervisor


class WebotsBinder(RobotBase):
    """You will receive updates in the next lab"""
    def __init__(self, robot_name, state_def, sensor_def, driver_def):
        super().__init__()
        # gets robot instance from the webots
        self._robot = Supervisor()
        self._robot_node = self._robot.getFromDef(robot_name)
        self._trans_field = self._robot_node.getField("translation")
        self._rotation_field = self._robot_node.getField("rotation")
        self.run.DT = self._timestep / 1000
        # add drivers and sensors
        self.drivers.append(WebotsDrive(self._robot, driver_def))
        self.sensors.append(WebotsSense(self._robot, self._timestep, sensor_def))



    def drive(self, inpts, timestamp):
        """drive the robot to the next state
        :param inpts: left, right wheel velocities
        :return full state feedback"""
        self.inpt = inpts
        self.driver.drive(inpts)
        return self.state

    def get_state(self):
        self.state = self.wb_sense.state
        return

    def sense(self):
        """generate the sensor reading"""
        self.outpt = self.wb_sense.sense()
        return self.outpt


    def init(self, init_state=None):
        pass




    def _init_webots(self):

        self._trans_field.setSFVec3f(self.state[0:2] + [0.0])    #move robot to the init state
        self._rotation_field.setSFRotation([0, 1, 0, self.state[2]])
        self._robot_node.setVelocity([0,0,0,0,0,0])
        self._robot_node.resetPhysics()             #reset physics
        # get the time step of the current world.
        self._timestep = int(self._robot.getBasicTimeStep())
        # init motors and sensors
        self._robot.step(self._timestep)
