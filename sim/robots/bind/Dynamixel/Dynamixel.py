import dynamixel_sdk as x
from .constants import XM430

import logging

class Dynamixel:
    def __init__(self, device_port, ids):
        self.port = x.PortHandler(device_port)
        self.device_port = device_port
        self.packet = x.PacketHandler(XM430.PROTOCOL_VERSION)
        self.ids = ids
        self.enabled_ids = [False] * len(ids)


    def init(self):
        if self.open_device():
            self.enable(enable=True)


    def open(self):
        if not self.port.is_open:
            self.port.openPort()
            self.port.setBaudRate(XM430.BAUDRATE)

        if self.port.is_open:
            logging.info('Dynamixel Connected')
            return True
        else:
            logging.error('Connection failed to {}'.format(self.device_port))
        return False


    def close(self):
        if self.port.is_open:
            self.enable(False)
            self.port.closePort()
            logging.info('Dynamixel port closed')

    def enable(self, enable):
        if self.port.is_open:
            for i, id in enumerate(self.ids):
                result, error = self.packet.write1ByteTxRx(self.port, id, XM430.TORQUE_ENABLE.ADDR, enable)
                if result == x.COMM_SUCCESS:
                    self.enabled_ids[i] = True

            if sum(self.enabled_ids) == len(self.ids):
                return True
        return False


    def __del__(self):
        self.close()
