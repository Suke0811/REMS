from rems.device import DriveBase
from defdict import DefDict
from defdict.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType


class WebotsDrive(DriveBase):
    device_name = 'Webots'
    def __init__(self, wb_robot):
        super().__init__()

        self._robot = wb_robot
        self.to_thread = False

    def init(self, *args, **kwargs):
        self._motors = DefDict(self.drive_space.list_keys())

    def open(self):
        for motor_name in self._motors.keys():
            motor = self._robot.getDevice(motor_name)
            self._motors[motor_name] = motor
        self.enable(True)

    def enable(self, enable, *args, **kwargs):
        for drive in self.drive_space:
            drive['on'] = True

    def drive(self, inpt, timestamp, *args, **kwargs):
        self.drive_space.set(inpt)
        # send rotational velocity to each motor
        for joint, motor in zip(self.drive_space, self._motors):
            if joint.get('pos') == float('inf'):
                self._set_velocity_mode(motor)
                motor.setVelocity(joint.get('vel'))
            else:
                motor.setPosition(joint.get('pos'))

    @staticmethod
    def _set_velocity_mode(motor):
        motor.setPosition(float('inf'))

    @staticmethod
    def create_drive_space(driver_names, *args, **kwargs):
        motor = dict(pos=Ang, vel=AngVel, acc=AngAcc, on=bool)
        return DefDict(driver_names, dtype=motor)

