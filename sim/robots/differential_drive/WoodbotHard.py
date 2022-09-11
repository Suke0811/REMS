from sim.robots import RobotBase
from sim.device.connect.WifiDevice import WifiDevice
from sim.device.connect.WebsocketDevice import WebsocketDevice
from sim.device.state_estimator.ArucoDevice import ArucoDevice
from sim.utils import time_str
import websocket, struct
from sim.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType, Percent
from sim.typing import DefDict
TARGET = "ws://192.168.4.1:81"


class Fs90r(Percent):
    default_unit = 'percent'
    default_dtype = float
    # data scale. (-1, 1) -> -100% to 100%. (0, 1) -> 0% to 100%
    defualt_drange_scale = (-1, 1)


class WoodbotHard(RobotBase):
    def __init__(self, target_address=TARGET, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.run.DT = 0.2
        self.target = target_address
        self.to_thread = False

    def init(self, *args, **kwargs):
        self.dev_inpt = DefDict(dict(wh_l=Fs90r, wh_r=Fs90r))
        self.ws = websocket.WebSocket()
        self.open()

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
        self.dev_inpt.set(inpt)
        print(inpt,  self.dev_inpt)
        cmd = [126] + [int(90 * x + 90) for x in self.dev_inpt.values()]
        self.ws.send(bytes(cmd), websocket.ABNF.OPCODE_BINARY)

    def sense(self, *args, **kwargs):
        self.ws.send("#S")
        resp_opcode, msg = self.ws.recv_data()
        sensors = struct.unpack("<HH", msg)
        self.outpt.update([float(x) for x in sensors])
        return self.outpt


if __name__ == '__main__':
    class WoodbotVel(AngVel):
        default_unit = 'rad/s'
        default_value = None
        default_dtype = float
        default_drange = (-7, 7)
        default_drange_map = None
        # data scale. (-1, 1) -> -100% to 100%. (0, 1) -> 0% to 100%
        defualt_drange_scale = (-1, 1)
    DRIVE = {
        "motor_l": WoodbotVel,
        "motor_r": WoodbotVel
    }
    d = DefDict(DRIVE)
    b = DefDict(dict(motor_l=Fs90r, motor_r=Fs90r))
    d.set([5,10])
    b.set(d)
    d.set(b)
    pass
