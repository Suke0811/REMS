from sim.robots.bind import DriveBase
from sim.type import DefDict
from sim.type import definitions as DEF


class WebotsDrive(DriveBase):
    def __init__(self, wb_robot, motor_definition: DefDict):
        super().__init__()
        self._motors = motor_definition
        self.inpt = motor_definition
        self._robot = wb_robot

    def open(self):
        for motor_name, motor in self._motors.definition.items():
            motor = self._robot.getDevice(motor_name)

    def enable(self, enable):
        for type_, motor in zip(self._motors.definition.as_list(), self._motors.data.as_list()):
            if isinstance(type_, DEF.velocity):
                self._set_velocity_mode(motor)

    def drive(self, inpt:DefDict):
        self.inpt = inpt
        # send rotational velocity to each motor
        for type_, value, motor in zip(self.inpt.definition.as_list(), self.inpt.data.as_list(), self._motors.data):
            if isinstance(type_, DEF.velocity):
                self._set_velocity_mode(motor)
                motor.setVelocity(value)
            else:
                motor.setPosition(value)

    @staticmethod
    def _set_velocity_mode(motor):
        motor.setPosition(float('inf'))

