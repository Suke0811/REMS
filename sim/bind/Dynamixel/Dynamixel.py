from sim.robots.DeviceBase import DeviceBase
from sim.typing import DefDict
from sim.typing import definitions as DEF
from sim.bind.Dynamixel.DynamixelTable import DynamiexX
from sim.typing import BindRule as rule
import logging, time
import numpy as np
import dynamixel_sdk as x

DEFAULT_SPEED = 2
DEFAULT_ACC = 3
ID = 'ID'
def dynamixel_id(id_list, dtype, prefix=''):
    return DEF.define(prefix, id_list, dtype)

def dynamixel_sensor_def(driver_def: DefDict):
    d = DefDict(dict(j=DEF.angular_position, d_j=DEF.angular_velocity))
    ret = DefDict(driver_def.list(), d)
    return d

MOTOR = DefDict(dict(pos=np.pi, vel=DEFAULT_SPEED, acc=DEFAULT_ACC, on=False))
def define_motor(id_lists):
    d = DEF.define(prefix=ID, num=id_lists, type_=MOTOR)
    return DefDict(d, name='dynamixel', prefixes=ID, suffixes=MOTOR.list_keys())

default_func = (None, None)

class Dynamixel(DeviceBase):
    def __init__(self, id_lists, slave_ids=None, device_port='/dev/ttyUSB0', offset_func=default_func):
        self.device_port = device_port
        self.packet = x.PacketHandler(DynamiexX.PROTOCOL_VERSION)

        self.ids = id_lists
        if slave_ids and slave_ids is not None:
            self.ids.extend(slave_ids)

        # motor sensing is only for masters
        self.motors_outpt = define_motor(id_lists)
        # motor input is for all
        self.motors_inpt = define_motor(id_lists)
        self.slave_bind = rule(DEF.define(prefix=ID, num=id_lists),
                               None,
                               DEF.define(prefix=ID, num=slave_ids))

        self.toDynamixel = rule(None, offset_func[0])
        self.fromDynamixel = rule(self.motors_outpt.pos().list_keys(), offset_func[1])

        self.read_vel = False

    def init(self):
        self.port = x.PortHandler(self.device_port)
        if self.open():
            self.restart()
            self.enable(enable=True)
            self._sync_write(self.motors_inpt.ID(), DynamiexX.OPERATING_MODE, DynamiexX.OPERATING_MODE.POS_MODE)
            self._sync_write(self.motors_inpt.ID().vel(), DynamiexX.PROFILE_VELOCITY)
            self._sync_write(self.motors_inpt.ID().acc(), DynamiexX.PROFILE_ACCELERATION)

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

    def homing(self):
        self._sync_write(self.motors_inpt.ID().pos(), DynamiexX.GOAL_POSITION, np.pi)
        time.sleep(1)   # TODO: wait till the motor reach the target position

    def enable(self, enable):
        if self.port.is_open:
            for i, id in enumerate(self.ids):
                result = self.func_retry(self.packet.write1ByteTxRx,
                                         args=(self.port, id, DynamiexX.TORQUE_ENABLE.ADDR, enable),
                                         success_condition=x.COMM_SUCCESS)
                if result == x.COMM_SUCCESS:
                    self.motors_inpt.ID(id).on().set(enable)
            if sum(self.motors_inpt.on().list()) == len(self.ids):
                logging.info(f'enabled {self.motors_inpt.on()}')
                return True
            else:
                if enable:
                    logging.info(f'Could not enabled {self.motors_inpt.on()}')
                    raise ConnectionError(f"Dynamixel could not enabled {self.motors_inpt.on()}")
                else:
                    logging.info('Disabled')
        return False

    def drive(self, inpt: DefDict, timestamp):
        # TODO: change this so that each motor could have different mode
        self.motors_inpt.set(self.slave_bind.bind(inpt))
        self._sync_write(self.motors_inpt.ID().vel(), DynamiexX.PROFILE_VELOCITY)
        self._sync_write(self.motors_inpt.pos().bind(self.toDynamixel).ID(), DynamiexX.GOAL_POSITION)

    def sense(self):
        if not self.read_vel:
            self._sync_read(self.motors_outpt.ID().pos(), DynamiexX.PRESENT_POSITION)
            self.motors_outpt.pos().bind(self.fromDynamixel)
            self.read_vel = True
        else:
            self._sync_read(self.motors_outpt.ID().vel(), DynamiexX.PRESENT_VELOCITY)
            self.read_vel = False
        return self.motors_outpt

    def __del__(self):
        self.close()

    def _sync_write(self, values:DefDict, table, value_overwrite=None):
        # TODO use bulk instead (table def added when add params)
        write = x.GroupSyncWrite(self.port, self.packet, table.ADDR, table.LEN)
        for id, val in zip(values.list_keys(), values.list()):
            if value_overwrite is not None: val = value_overwrite
            write.addParam(int(id), table.from_unit(val))
        result = self.func_retry(write.txPacket, success_condition=x.COMM_SUCCESS)
        write.clearParam()

    def _sync_read(self, values: DefDict, table):
        # TODO use bulk instead (table def added when add params)
        read = x.GroupSyncRead(self.port, self.packet, table.ADDR, table.LEN)
        for id, val in zip(values.list_keys(), values.list()):
            read.addParam(int(id))
        read.txRxPacket()
        #self.func_retry(read.txRxPacket, success_condition=x.COMM_SUCCESS)
        for id, key in zip(values.list_keys(), values.list_keys()):
            result = read.isAvailable(int(id), table.ADDR, table.LEN)
            if result:
                # Get Dynamixel present position value
                values[key] = table.to_unit(read.getData(int(id), table.ADDR, table.LEN))
        read.clearParam()
        return values

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


