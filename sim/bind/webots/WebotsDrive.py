from sim.bind import DriveBase
from sim.typing import DefDict
from sim.typing import definitions as DEF


class WebotsDrive(DriveBase):
    def __init__(self, wb_robot, motor_definition: DefDict):
        super().__init__()
        self._motors = DefDict(motor_definition.list_keys())
        self.joint_space = motor_definition.clone()
        self._robot = wb_robot

    def open(self):
        for motor_name in self._motors.keys():
            motor = self._robot.getDevice(motor_name)
            self._motors[motor_name] = motor

    def enable(self, enable):
        for joint in self.joint_space:
            joint['on'] = True


    def drive(self, jointspace: DefDict, timestamp):
        self.joint_space.set(jointspace)
        # send rotational velocity to each motor
        for joint, motor in zip(self.joint_space, self._motors):
            if joint.get('pos') == float('inf'):
                self._set_velocity_mode(motor)
                motor.setVelocity(joint.get('vel'))
            else:
                motor.setPosition(joint.get('pos'))

    @staticmethod
    def _set_velocity_mode(motor):
        motor.setPosition(float('inf'))

