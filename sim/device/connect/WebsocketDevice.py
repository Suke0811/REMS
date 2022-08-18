from sim.device.RobotBase import RobotBase
from sim.device.DeviceBase import DeviceBase
import websocket, struct

TARGET = "ws://192.168.4.1:81"
# ESP_00111404

class WebsocketDevice(DeviceBase):
    def __init__(self, target_address=TARGET):
        super().__init__()
        self.target = target_address

    def init(self, *args, **kwargs):
        self.ws = websocket.WebSocket()
        self.open()

    def enable(self, enable: bool):
        pass

    def open(self):
        self.ws.connect(self.target)
        print(self.ws.recv())  # "Connected to"
        print(self.ws.recv())  # "ESP_xxxx"

    def close(self):
        self.ws.send("#0")
        self.ws.close()

    def drive(self, inpt_cmd, timestamp):
        cmd = [126] + inpt_cmd
        self.ws.send(bytes(cmd), websocket.ABNF.OPCODE_BINARY)

    def sense(self):
        self.ws.send("#S")
        resp_opcode, msg = self.ws.recv_data()
        sensors = struct.unpack("<HH", msg)
        return [float(x) for x in sensors]


