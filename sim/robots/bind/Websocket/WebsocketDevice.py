from sim.robots.bind import FullDeviceBase
import websocket, struct

TARGET = "ws://192.168.4.1:81"

class Websocket(FullDeviceBase):
    def __init__(self, target_address=TARGET):
        super().__init__()
        self.target = target_address
        self.ws = websocket.WebSocket()

    def init(self):
        pass

    def open(self):
        self.ws.connect(self.target)
        print(self.ws.recv())  # "Connected to"
        print(self.ws.recv())  # "ESP_xxxx"

    def close(self):
        self.ws.send("#0")
        self.ws.close()

    def drive(self, inpt, timestamp):
        cmd = [126] + [int(90 * x + 90) for x in inpt]
        self.ws.send(bytes(cmd), websocket.ABNF.OPCODE_BINARY)

    def sense(self, definition=None):
        self.ws.send("#S")
        resp_opcode, msg = self.ws.recv_data()
        sensors = struct.unpack("<HH", msg)
        self.outpt = [float(x) for x in sensors]
        return self.outpt

