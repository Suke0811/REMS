import dynamixel_sdk as x
from sim.robots.bind.Dynamixel.constants import *
from sim.robots.DeviceBase import DeviceBase
from sim.type import DefDict
from sim.type import definitions as DEF

import logging


class Dynamixel(DeviceBase):
    def __init__(self, ids):
        self.device_port = 'COM3'
        self.packet = x.PacketHandler(XM430.PROTOCOL_VERSION)
        self.ids = ids
        self.enabled_ids = [False] * len(ids)

    def init(self):
        self.port = x.PortHandler(self.device_port)
        if self.open():
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
                for retry_count in range(XM430.RETRY_MAX):
                    print(f"port {self.port.is_using}")
                    result, error = self.packet.write1ByteTxRx(self.port, id, XM430.TORQUE_ENABLE.ADDR, enable)
                    print(result)
                    if result == x.COMM_SUCCESS:
                        self.enabled_ids[i] = enable
                        break

            if sum(self.enabled_ids) == len(self.ids):
                logging.info('enabled {}'.format(self.enabled_ids))
                return True
            else:
                logging.info('Could not enabled {}'.format(self.enabled_ids))

        return False

    def drive(self, inpt:DefDict, timestamp):
        for type_, value, motor in zip(self.inpt.DEF.as_list(), inpt.data.as_list(), self._motors.data):
            if isinstance(type_, DEF.velocity):
                self._set_velocity_mode(motor)
                motor.setVelocity(value)
            else:
                motor.setPosition(value)


    def sense(self, timestamp):
        raise NotImplementedError

    def observe_state(self):
        raise NotImplementedError

    def __del__(self):
        self.close()

    def _sync_write(self, values:DefDict, table):
        # TODO use bulk instead (table def added when add params)
        write = x.GroupSyncWrite(self.port, self.packet, table.ADDR, table.LEN)
        for id, val in values.data.items():
            write.addParam(int(id), table.from_unit(val))
        result = self.func_retry(write.txPacket, success_condition=x.COMM_SUCCESS)
        print("%s" % self.packet.getTxRxResult(result))
        write.clearParam()

    def _sync_read(self, values:DefDict, table):
        read = x.GroupSyncRead(self.port, self.packet, table.ADDR, table.LEN)
        for id, val in values.data.items():
            read.addParam(int(id))

        self.func_retry(read.txRxPacket, success_condition=x.COMM_SUCCESS)

        for id, val in values.data.items():
            result = read.isAvailable(int(id), table.ADDR, table.LEN)
            print(result)
            if result:
                # Get Dynamixel present position value
                val = table.to_unit(read.getData(int(id), table.ADDR, table.LEN))
                print(val)
        read.clearParam()
        return values

    @staticmethod
    def func_retry(func, args=None, retry_max=XM430.RETRY_MAX, success_condition=True):
        for retry_count in range(retry_max):
            print(f"try{retry_count}")
            if args is None:
                result = func()
            else:
                result = func(*args)
            if result == success_condition:
                break
        return result


if __name__ == '__main__':
    import logging
    l = logging.getLogger()
    l.setLevel(logging.INFO)
    d = Dynamixel([1, 2])
    d.init()
    i = DefDict({'1': float, '2': float})
    i.data = [10, 10]

    d._sync_write(i, XM430.GOAL_POSITION)
    import time

    time.sleep(2)
    i.data = [True, True]
    d._sync_write(i, XM430.LED)
    #i.data = [0, 0]
    #d._sync_write(i, XM430.GOAL_POSITION)
    print(d._sync_read(i, XM430.PRESENT_POSITION).data.as_list())
    d.close()
    pass

