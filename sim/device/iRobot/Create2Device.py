from  pycreate2 import Create2
import time
from sim.device import DriveBase, SenseBase
from sim.typing import DefDict

dict(bumps_wheeldrops=float)


class Create2Device(DriveBase, SenseBase):
    def __init__(self, port, safety=False, *args, **keyword):
        super().__init__(*args, **keyword)
        self.device_name = 'Create 2'
        self.port = port
        self.safety = safety
        self.drive_space = DefDict(dict(wh_r=float, wh_l=float))
        self.sense_sapce = DefDict(dict(t=float))

    def open(self, *args, **kwargs):
        self.create = Create2(self.port)
        self.enable(True)

    def enable(self, enable, *args, **kwargs):
        if enable:
            self.create.start()
            if self.safety:
                self.create.safe()
            else:
                self.create.full()
        else:
            self.create.stop()

    def close(self, *args, **kwargs):
        self.enable(False)
        self.create.close()

    def drive(self, inpt, timestamp, *args, **kwargs):
        self.drive_space.update(inpt.vel().list())
        self.drive_space *= 20
        l, r = (int(i) for i in self.drive_space.list())
        self.create.drive_direct(l, r)

    def sense(self, *args, **kwargs):
        vals = self.create.get_sensors()
        print(vals)
        self.sense_sapce.set(vals)
        return self.sense_sapce


# # Create a Create2.
# port = "COM7"  # where is your serial port?
# bot = Create2(port)
#
# # Start the Create 2
# bot.start()
#
# # Put the Create2 into 'safe' mode so we can drive it
# # This will still provide some protection
# bot.safe()
#
# # You are responsible for handling issues, no protection/safety in
# # this mode ... becareful
# bot.full()
#
# # directly set the motor speeds ... move forward
# bot.drive_direct(100, 100)
# time.sleep(2)
#
# # turn in place
# bot.drive_direct(200,-200)  # inputs for motors are +/- 500 max
# time.sleep(2)
#
# # Stop the bot
# bot.drive_stop()
#
# # query some sensors
# sensors = bot.get_sensors()  # returns all data
# print(sensors.light_bumper_left)
#
# # Close the connection
# # bot.close()
