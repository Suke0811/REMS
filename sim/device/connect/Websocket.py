from sim.device.RobotBase import RobotBase
import websocket, struct

TARGET = "ws://192.168.4.1:81"
# ESP_00111404

class Websocket(RobotBase):
    def __init__(self, target_address=TARGET):
        super().__init__()
        self.target = target_address
        self.run.DT = 0.25
        self.run.name += 'web'


    def init(self, *args, **kwargs):
        self.ws = websocket.WebSocket()
        self.open()

    def open(self):
        self.ws.connect(self.target)
        print(self.ws.recv())  # "Connected to"
        print(self.ws.recv())  # "ESP_xxxx"

    def close(self):
        self.ws.send("#0")
        self.ws.close()

    def drive(self, inpt, timestamp):
        cmd = [126] + [int(90 * x/16 + 90) for x in inpt.vel()]
        self.ws.send(bytes(cmd), websocket.ABNF.OPCODE_BINARY)

    def sense(self, definition=None):
        self.ws.send("#S")
        resp_opcode, msg = self.ws.recv_data()
        sensors = struct.unpack("<HH", msg)
        self.outpt.set([float(x) for x in sensors])
        return self.outpt

