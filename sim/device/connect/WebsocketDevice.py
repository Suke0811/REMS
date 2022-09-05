from sim.device.DeviceBase import DeviceBase
import websocket, struct

TARGET = "ws://192.168.4.1:81"
# ESP_00111404

class WebsocketDevice(DeviceBase):
    def __init__(self, target_address=TARGET, *args, **kwargs):
        super().__init__(*args, **kwargs)
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
        cmd = [126] + [int(90 * x + 90) for x in inpt.vel()]
        self.ws.send(bytes(cmd), websocket.ABNF.OPCODE_BINARY)

    def sense(self, *args, **kwargs):
        self.ws.send("#S")
        resp_opcode, msg = self.ws.recv_data()
        sensors = struct.unpack("<HH", msg)
        return [float(x) for x in sensors]


