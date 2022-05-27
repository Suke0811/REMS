from sim.robots.bind.Dynamixel.DynamixelTable import *
from sim.robots.DeviceBase import DeviceBase
from sim.type import DefDict
from sim.type import definitions as DEF
import time
import dynamixel_sdk as x
from serial.serialutil import SerialException
import logging
from sim.utils.tictoc import tictoc

DEFAULT_SPEED = 2
DEFAULT_ACC = 20
ID = 'id'
def dynamixel_id(id_list, dtype, prefix=''):
    return DEF.define(prefix, id_list, dtype, separater='')


def dynamixel_sensor_def(driver_def: DefDict):
    d = DefDict(dict(j=DEF.angular_position, d_j=DEF.angular_velocity))
    ret = DefDict(driver_def.data.as_list(), d)
    return ret


class Dynamixel(DeviceBase):
    def __init__(self, id_list, slave_ids=None, device_port='COM3'):
        self.device_port = device_port
        self.packet = x.PacketHandler(DynamiexX.PROTOCOL_VERSION)
        self.ids = [int(i) for i in id_list]
        if slave_ids and slave_ids is not None:
            self.ids.extend([int(i) for i in slave_ids])
        self.motors = DefDict(dynamixel_id(self.ids, DEF.angular_position))
        # TODO chage to use nested DefDict?
        self.motor_pos = DefDict(dynamixel_id(id_list, DEF.angular_position,'j.'))
        self.motor_vel = DefDict(dynamixel_id(id_list, DEF.angular_velocity, 'd_j.'))
        self.motor_sensors = DefDict(dynamixel_id(id_list, DEF.angular_position,'j.'), dynamixel_id(id_list, DEF.angular_velocity, 'd_j.'))
        self.enabled_ids = {k: False for k, v in self.motors.data.items()}  # TODO: change to use DefDict

    def init(self):
        self.port = x.PortHandler(self.device_port)
        if self.open():
            self.restart()
            self.enable(enable=True)
            self.read = x.GroupBulkRead(self.port, self.packet)
            self._sync_write(self.motors, DynamiexX.OPERATING_MODE, DynamiexX.OPERATING_MODE.POS_MODE)
            self._sync_write(self.motors, DynamiexX.PROFILE_VELOCITY, DEFAULT_SPEED)
            self._sync_write(self.motors, DynamiexX.PROFILE_ACCELERATION, DEFAULT_SPEED)


    def open(self):
        if not self.port.is_open:
            self.port.openPort()
            self.port.setBaudRate(DynamiexX.BAUDRATE)

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
                                         args=(self.port, id, DynamiexX.TORQUE_ENABLE.ADDR, enable),
                                         success_condition=x.COMM_SUCCESS)
                if result == x.COMM_SUCCESS:
                    self.enabled_ids[str(id)] = enable
            if sum(self.enabled_ids.values()) == len(self.ids):
                logging.info('enabled {}'.format(self.enabled_ids))
                return True
            else:
                if enable:  logging.info('Could not enabled {}'.format(self.enabled_ids))
                else:       logging.info('Disabled')
        return False

    #@tictoc
    def drive(self, inpt: DefDict, timestamp):
        # TODO: change this so that each motor could have different mode
        self._sync_write(inpt, DynamiexX.PROFILE_VELOCITY, DEFAULT_SPEED)
        self._sync_write(inpt, DynamiexX.GOAL_POSITION)

    def sense(self):
        self._sync_read(self.motor_pos, DynamiexX.PRESENT_POSITION)
        self._sync_read(self.motor_vel, DynamiexX.PRESENT_VELOCITY)
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

    def _sync_read(self, values: DefDict, table):
        # TODO use bulk instead (table def added when add params)
        read = x.GroupSyncRead(self.port, self.packet, table.ADDR, table.LEN)
        for id, val in zip(values.data.get_key_suffix(), values.data.as_list()):
            read.addParam(int(id))
        # write
        apt=time.perf_counter()

        self.func_retry(read.txRxPacket, success_condition=x.COMM_SUCCESS)
        readt = time.perf_counter()
        #logging.info(f" read {readt - apt}")
        for id, key in zip(values.data.get_key_suffix(), values.data.key_as_list()):
            result = read.isAvailable(int(id), table.ADDR, table.LEN)
            if result:
                # Get Dynamixel present position value
                values.data[key] = table.to_unit(read.getData(int(id), table.ADDR, table.LEN))
        read.clearParam()
        return values

    def _bulk_read(self, values, tables):
        for table in tables:
            for id, val in zip(values.data.get_key_suffix(), values.data.as_list()):
                self.read.addParam(int(id), table.ADDR, table.LEN)
            self.func_retry(self.read.txRxPacket, success_condition=x.COMM_SUCCESS)
        readt = time.perf_counter()
        # logging.info(f" read {readt - apt}")
        for table in tables:
            for id, key in zip(values.data.get_key_suffix(), values.data.key_as_list()):
                result = self.read.isAvailable(int(id), table.ADDR, table.LEN)
                if result:
                    # Get Dynamixel present position value
                    values.data[key] = table.to_unit(self.read.getData(int(id), table.ADDR, table.LEN))
        self.read.clearParam()
        return values
        #self.read.

    def restart(self):
        # Try reboot
        # Dynamixel LED will flicker while it reboots
        for i in self.ids:
            self.func_retry(self.packet.reboot, args=(self.port, i), success_condition=x.COMM_SUCCESS)

    @staticmethod
    def func_retry(func, args=None, retry_max=DynamiexX.RETRY_MAX, success_condition=True):
        result = None
        for retry_count in range(retry_max):
            if args is None:    result = func()
            else:               result = func(*args)
            try:
                iterator = iter(result)
                result = next(iterator)
            except TypeError:
                pass
            if result == success_condition:
                logging.debug(f"Connection try {retry_count} succeed")
                break
        return result


if __name__ == '__main__':
    import logging
    l = logging.getLogger()
    l.setLevel(logging.INFO)
    d = Dynamixel([10,11,12,22,23,24], device_port='COM5')
    d.init()
    i = DefDict({'10': float, '11': float, '12':float, '22':float, '23':float, '24':float})
    i.data = [np.pi for _ in range(6)]
    d.drive(i, 0)
    import time

    # time.sleep(2)
    # d._sync_write(i, DynamiexX.PROFILE_VELOCITY, 5)
    # print(d._sync_read(i, DynamiexX.PRESENT_POSITION).data.as_list())
    # i.data = [True, True]
    # d._sync_write(i, DynamiexX.LED)
    # d._sync_write(i.set_data([False, False]), DynamiexX.LED)
    # d._sync_write(i, DynamiexX.GOAL_POSITION)
    print(d._sync_read(i, DynamiexX.PRESENT_POSITION).data.as_list())
    N = 500
    st = time.perf_counter()
    time.sleep(1)

    i.data = [np.pi for _ in range(6)]
    i.data = [3.13238716, 4.62341572, 4.33962942, 2.39147482, 3.1661347199999996, 3.1400570599999997]
    c = True
    for n in range(N):
        if n % 50 == 0:
            if c:
                i.data = [3.13238716, 4.62341572, 4.33962942, 2.39147482, 3.1661347199999996, 3.1400570599999997]
                c =False
            else:
                i.data = [np.pi for _ in range(6)]
                c =True
        d.drive(i, 0)
        d.sense()

    et = time.perf_counter()
    print(d._sync_read(i, DynamiexX.PRESENT_POSITION).data.as_list())
    print(et-st)
    print((et-st)/N)
    d.close()
    pass

