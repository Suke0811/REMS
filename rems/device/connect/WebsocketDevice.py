import time

from rems.device.DeviceBase import DeviceBase
import websocket, struct
from rems.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType, Percent
from rems.typing import DefDict
from rems.utils import tictoc

TARGET = "ws://192.168.4.1:81"
# ESP_00111404


class Fs90r(AngVel):
    default_unit = 'rad/s'
    default_dtype = float
    default_drange = (-6.8, 6.8)

class Fs90rCount(UnitType):
    default_unit = 'count'
    default_dtype = int
    default_drange = (150, 600)
    default_drange_map = ('-6.8rad/s', '6.8rad/s')


class Fs90rSend(Percent):
    default_unit = 'percent'
    default_dtype = float
    # data scale. (-1, 1) -> -100% to 100%. (0, 1) -> 0% to 100%
    defualt_drange_scale = (-1, 1)


class WebsocketDevice(DeviceBase):
    device_name = 'Woodbot'
    def __init__(self, target_address=TARGET, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.target = target_address
        self.to_thread = False # self.TO_THREAD
        self.config.on().set([True, False, False])

    def init(self, *args, **kwargs):
        self.dev_inpt = self.create_drive_space()
        self.drive_send = DefDict({'wh.r': Fs90rSend, 'wh.l': Fs90rSend})
        self.ws = websocket.WebSocket()

    def enable(self, enable: bool, *args, **kwargs):
        pass

    def open(self, *args, **kwargs):
        self.ws.connect(self.target)
        print(self.ws.recv())  # "Connected to"
        print(self.ws.recv())  # "ESP_xxxx"

    def close(self, *args, **kwargs):
        self.ws.send("#0")
        self.ws.close()

    def drive(self, inpt, timestamp, *args, **kwargs):
        self.dev_inpt.update(inpt)
        self.drive_send.update(self.dev_inpt)
        cmd = [126] + [int(90 * -x / 100 + 90) for x in self.drive_send.values()]
        self.ws.send(bytes(cmd), websocket.ABNF.OPCODE_BINARY)

    def sense(self, *args, **kwargs):
        self.ws.send("#S")
        resp_opcode, msg = self.ws.recv_data()
        sensors = struct.unpack("<HH", msg)
        self.sense_space.update([float(x) for x in sensors])
        return self.sense_space

    def create_drive_space(*args, **kwargs):
        return DefDict({'wh.l': Fs90r, 'wh.r': Fs90r})

    @staticmethod
    def create_sense_space(*args, **kwargs):
        return DefDict(dict(lidar_f=float, lidar_r=float, mag_x=float, mag_y=float, gyro_z=float))





