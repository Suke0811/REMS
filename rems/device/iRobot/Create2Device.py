from  pycreate2 import Create2
import time
from rems.device import DriveBase, SenseBase
from rems.typing import DefDict
from rems.typing.std.StdUnit import Count, Pos, Vel
from rems.utils import tictoc

class CountVel(Count):
    default_unit = 'count'
    default_dtype = int
    default_drange = (-500, 500)
    default_drange_map = ('-16 rad/s', '16 rad/s')
    defualt_drange_scale = (-1, 1)

sensor_def = dict(bumps_wheeldrops=int,
     wall=bool, cliff_left=bool, cliff_front_left=bool, cliff_front_right=bool, cliff_right=bool, virtual_wall=bool,
     overcurrents=int, dirt_detect=int, ir_opcode=int, buttons=int,
     distance=int, angle=int, charger_state=int, voltage=int, current=int, temperature=int,
     battery_charge=int, battery_capacity=int, wall_signal=int,
     cliff_left_signal=int, cliff_front_left_signal=int, cliff_front_right_signal=int, cliff_right_signal=int,
     charger_available=int, open_interface_mode=int, song_number=int, song_playing=int,
     oi_stream_num_packets=int, velocity=Vel, radius=Pos, velocity_right=CountVel, velocity_left=CountVel,
     encoder_counts_left=int, encoder_counts_right=int,
     light_bumper=int, light_bumper_left=int, light_bumper_front_left=int,
     light_bumper_center_left=int, light_bumper_center_right=int, light_bumper_front_right=int,
     light_bumper_right=int,
     ir_opcode_left=int, ir_opcode_right=int,
     left_motor_current=int, right_motor_current=int, main_brush_current=int, side_brush_current=int,
     statis=int)


class Create2Device(DriveBase, SenseBase):
    device_name = 'Create2Device'

    def __init__(self, port, safety=False, *args, **keyword):
        super().__init__(*args, **keyword)
        self.config.on().set([True, True, False])
        self.to_thread = self.TO_THREAD
        self.port = port
        self.safety = safety

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
        self.drive_space.update(inpt)
        l, r = self.drive_space.list()
        self.create.drive_direct(r, l)

    def sense(self, *args, **kwargs):
        #return self.sense_space
        #return self.sense_space
        vals = self.create.get_sensors()
        self.sense_space.set(vals)
        return self.sense_space

    @classmethod
    def create_drive_space(cls, *args, **kwargs):
        return DefDict({'wh.l': CountVel, 'wh.r': CountVel})

    @classmethod
    def create_sense_space(cls, *args, **kwargs):
        return DefDict(sensor_def)
