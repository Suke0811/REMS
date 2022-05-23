import dynamixel_sdk as x
from sim.robots.bind.Dynamixel.constants import *
from sim.robots.DeviceBase import DeviceBase
from sim.type import DefDict
from sim.type import definitions as DEF
import logging

ID = 'id'
def dynamixel_id(id_list, dtype, prefix=''):
    return DEF.define(prefix, id_list, dtype)


def dynamixel_sensor_def(driver_def: DefDict):
    d = DefDict(dict(j=DEF.angular_position, d_j=DEF.angular_velocity))
    ret = DefDict(driver_def.data.as_list(), d)
    return ret


class Dynamixel(DeviceBase):
    def __init__(self, id_list, slave_ids=None, device_port='COM3'):
        self.device_port = device_port
        self.packet = x.PacketHandler(XM430.PROTOCOL_VERSION)
        self.ids = id_list
        if slave_ids:
            self.ids.update(slave_ids)
        self.motors = DefDict(dynamixel_id(self.ids, DEF.angular_position))
        # TODO chage to use nested DefDict?
        self.motor_pos = DefDict(dynamixel_id(id_list, DEF.angular_position,'j.'))
        self.motor_vel = DefDict(dynamixel_id(id_list, DEF.angular_velocity, 'd_j.'))
        self.motor_sensors = DefDict(dynamixel_id(id_list, DEF.angular_position,'j.'), dynamixel_id(id_list, DEF.angular_velocity, 'd_j.'))

        self.enabled_ids = {k: False for k, v in self.motors.data.items()}  # TODO: change to use DefDict
        self.port = None

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
                result = self.func_retry(self.packet.write1ByteTxRx,
                                         args=(self.port, id, XM430.TORQUE_ENABLE.ADDR, enable),
                                         success_condition=x.COMM_SUCCESS)
                if result == x.COMM_SUCCESS:    self.enabled_ids[i] = enable
            if sum(self.enabled_ids) == len(self.ids):
                logging.info('enabled {}'.format(self.enabled_ids))
                return True
            else:
                if enable:  logging.info('Could not enabled {}'.format(self.enabled_ids))
                else:       logging.info('Disabled')
        return False

    def drive(self, inpt: DefDict, timestamp):
        # TODO: change this so that each motor could have different mode
        if isinstance(inpt.DEF.as_list()[0], DEF.velocity):
            self._sync_write(inpt, XM430.OPERATING_MODE, XM430.OPERATING_MODE.POS_MODE)
            self._sync_write(inpt, XM430.GOAL_VELOCITY)
        else:
            self._sync_write(inpt, XM430.OPERATING_MODE, XM430.OPERATING_MODE.POS_MODE)
            self._sync_write(inpt, XM430.GOAL_POSITION)

    def sense(self):
        self._sync_read(self.motor_pos, XM430.PRESENT_POSITION)
        self._sync_read(self.motor_vel, XM430.PRESENT_VELOCITY)
        self.motor_sensors.data = self.motor_pos
        self.motor_sensors.data = self.motor_vel
        return self.motor_sensors

    def __del__(self):
        self.close()

    def _sync_write(self, values:DefDict, table, value_overwrite=None):
        # TODO use bulk instead (table def added when add params)
        write = x.GroupSyncWrite(self.port, self.packet, table.ADDR, table.LEN)
        for id, val in zip(values.data.get_key_suffix(), values.data.as_list()):
            if value_overwrite is not None: val = value_overwrite
            write.addParam(int(id), table.from_unit(val))
        result = self.func_retry(write.txPacket, success_condition=x.COMM_SUCCESS)
        write.clearParam()

    def _sync_read(self, values:DefDict, table):
        # TODO use bulk instead (table def added when add params)
        read = x.GroupSyncRead(self.port, self.packet, table.ADDR, table.LEN)
        for id, val in values.data.items():
            read.addParam(int(id))
        # write
        self.func_retry(read.txRxPacket, success_condition=x.COMM_SUCCESS)
        for id, val in zip(values.data.get_key_suffix(), values.data.as_list()):
            result = read.isAvailable(int(id), table.ADDR, table.LEN)
            if result:
                # Get Dynamixel present position value
                val = table.to_unit(read.getData(int(id), table.ADDR, table.LEN))
        read.clearParam()
        return values

    @staticmethod
    def func_retry(func, args=None, retry_max=XM430.RETRY_MAX, success_condition=True):
        result = None
        for retry_count in range(retry_max):
            logging.debug(f"Connection try {retry_count} of {retry_max}")
            if args is None:
                result = func()
            else:
                result = func(*args)
            try:
                iterator = iter(result)
                result = next(iterator)
            except TypeError:
                pass
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
    i.data = [3.14, 3.14]

    d._sync_write(i, XM430.GOAL_POSITION)
    import time

    time.sleep(2)
    print(d._sync_read(i, XM430.PRESENT_POSITION).data.as_list())
    i.data = [True, True]
    d._sync_write(i, XM430.LED)
    i.data = [0, 0]
    d._sync_write(i.set_data([False, False]), XM430.LED)
    d._sync_write(i, XM430.GOAL_POSITION)
    print(d._sync_read(i, XM430.PRESENT_POSITION).data.as_list())
    time.sleep(1)
    d.close()
    pass

