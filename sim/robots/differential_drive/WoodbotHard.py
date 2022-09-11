from sim.robots import RobotBase
from sim.device.connect.WifiDevice import WifiDevice
from sim.device.connect.WebsocketDevice import WebsocketDevice
from sim.device.state_estimator.ArucoDevice import ArucoDevice
from sim.utils import time_str
import websocket, struct

TARGET = "ws://192.168.4.1:81"


class WoodbotHard(RobotBase):
    def __init__(self, target_address=TARGET, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.run.DT = 0.2
        self.target = target_address
        self.to_thread = False

    def init(self, *args, **kwargs):
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
        cmd = [126] + [int(90 * x + 90) for x in inpt]
        self.ws.send(bytes(cmd), websocket.ABNF.OPCODE_BINARY)

    def sense(self, *args, **kwargs):
        self.ws.send("#S")
        resp_opcode, msg = self.ws.recv_data()
        sensors = struct.unpack("<HH", msg)
        self.outpt.update([float(x) for x in sensors])
        return self.outpt

