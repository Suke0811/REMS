import copy

from rems.device.DeviceBase import DeviceBase
from rems.typing import DefDict
from rems.typing.std import StdDefinitions as DEF
from rems.device.Dynamixel.DynamixelTable import DynamixelX
from rems.typing import MapRule as rule
from rems.typing.std.StdUnit import Count, Pos, Vel, Acc, Ang, AngVel, AngAcc
from rems.utils import tictoc
import logging, time
import numpy as np
from rems.device.Dynamixel import dynamixel_sdk as x

DEFAULT_SPEED = 6.0
DEFAULT_ACC = 17.5
ID = 'ID'

default_func = (None, None)

motor = DefDict(dict(pos=float,#Ang,
             vel=float,#AngVel(unit='rad/s', drange=('-52rpm', '52rpm'), default=DEFAULT_SPEED),
             acc=float,#Acc(default=DEFAULT_ACC),
             on=bool))

sensor = DefDict(dict(pos=float,#Ang,
                        vel=float,))#AngVel(unit='rad/s', drange=('-52rpm', '52rpm'))))


class Dynamixel(DeviceBase):
    device_name = 'Dynamixel'
    def __init__(self, id_lists, slave_ids=None, device_port='/dev/ttyUSB0', offset_func=default_func, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.realtime = 0.0

        self.device_port = device_port
        # motor sensing is only for masters
        # motor input is for all
        self.ids = id_lists
        self.slave_ids = slave_ids
        self.offset_func = offset_func
        self.id_lists = copy.deepcopy(id_lists)
        self.config.step().update([0.02,0.02,0.01])
        slave_bind = None

        # flags i dont like
        self.read_vel = False
        self.vel_mode = False
        self.to_thread = True

    def add_slave_rule(self):
        if self.slave_ids and self.slave_ids is not None:
            m = []
            s = []
            for pair in self.slave_ids:
                m.append(pair[0])
                s.append(pair[1])
            self.ids.extend(s)
            try:
                [self.id_lists.remove(slave) for slave in s]
            except ValueError:
                pass
            self.slave_ids = s
            slave_bind = rule(DEF.define(prefix=ID, num=m, dtype=motor),
                                   None,
                                   DEF.define(prefix=ID, num=s, dtype=motor), to_list=True)
            self.slave_bind = slave_bind
            self.drive_space.set_rule(slave_bind)

    def init(self, *args, **kwargs):
        self.add_slave_rule()
        self.toDynamixel = rule(DEF.define(prefix=ID, num=self.id_lists), self.offset_func[0], to_list=True)
        self.fromDynamixel = rule(self.sense_space.pos().list_keys(), self.offset_func[1], to_list=True)
        self.packet = x.PacketHandler(DynamixelX.PROTOCOL_VERSION)
        self.port = x.PortHandler(self.device_port)

    def open(self, *args, **kwargs):
        if not self.port.is_open:
            self.port.openPort()
            self.port.setBaudRate(DynamixelX.BAUDRATE)
        if self.port.is_open:
            logging.info('Dynamixel Connected')
            opened = True
        else:
            logging.error('Connection failed to {}'.format(self.device_port))
            opened = False

        if opened:
            self.restart()
            self.enable(enable=True)
            self.velocity_mode()
            for key, acc in self.drive_space.acc().items():
                if acc == 0:
                    self.drive_space.acc()[key] = DEFAULT_ACC
                    self.drive_space.vel()[key] = DEFAULT_SPEED
            self._sync_write(self.drive_space.ID().acc(), DynamixelX.PROFILE_ACCELERATION)
            self._sync_write(self.drive_space.ID().vel(), DynamixelX.PROFILE_VELOCITY)

    def close(self, *args, **kwargs):
        if self.port.is_open:
            self.enable(False)
            self.port.closePort()
            logging.info('Dynamixel port closed')

    def homing(self):
        self._sync_write(self.drive_space.ID().pos(), DynamixelX.GOAL_POSITION, np.pi)
        time.sleep(1)   # TODO: wait till the motor reach the target position

    def enable(self, enable, *args, **kwargs):
        time.sleep(0.1)
        if self.port.is_open:
            for i, id in enumerate(self.ids):
                result = self.func_retry(self.packet.write1ByteTxRx,
                                         args=(self.port, id, DynamixelX.TORQUE_ENABLE.ADDR, enable),
                                         success_condition=x.COMM_SUCCESS)
                if result == x.COMM_SUCCESS:
                    self.drive_space.ID(id).on().set(enable)
            if sum(self.drive_space.on().list()) == len(self.drive_space):
                logging.info(f'enabled {self.drive_space.on()}')
                return True
            else:
                if enable:
                    logging.info(f'Could not enabled {self.drive_space.on()}')
#                    raise ConnectionError(f"Dynamixel could not enabled {self.drive_space.on()}")
                else:
                    logging.info('Disabled')
        return False

    def drive(self, inpt: DefDict, timestamp, *args, **kwargs):
        # TODO: change this so that each motor could have different mode
        self.drive_space.set(inpt)
        if self.velocity_mode():
            self._sync_write(self.drive_space.ID().vel(), DynamixelX.GOAL_VELOCITY)
        else:
            self.drive_space.pos().bind(self.toDynamixel)
            if timestamp <= 0.0:
                return
            val = self.drive_space.bind(self.slave_bind).pos().ID()
            self._sync_write(val, DynamixelX.GOAL_POSITION)
            #self._sync_write(self.drive_space.ID().vel(), DynamixelX.PROFILE_VELOCITY)

    def sense(self, *args, **kwargs):
        if True: # not self.read_vel and not self.vel_mode:
            self._sync_read(self.sense_space.ID().pos(), DynamixelX.PRESENT_POSITION)
            self.sense_space.pos().bind(self.fromDynamixel)
            self.read_vel = True
        else:
            self._sync_read(self.sense_space.ID().vel(), DynamixelX.PRESENT_VELOCITY)
            self.read_vel = False
        #print(self.sense_space)
        return self.sense_space

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


    def velocity_mode(self):
        return self.vel_mode
        if self.vel_mode is False and any(self.drive_space.pos() == float('inf')):
            self._sync_write(self.drive_space.ID(), DynamixelX.OPERATING_MODE, DynamixelX.OPERATING_MODE.VEL_MODE)
            self.vel_mode = True
        elif self.vel_mode is False:
            self._sync_write(self.drive_space.ID(), DynamixelX.OPERATING_MODE, DynamixelX.OPERATING_MODE.POS_MODE)
            self._sync_write(self.drive_space.ID().vel(), DynamixelX.PROFILE_VELOCITY)
            self.vel_mode = False
        return self.vel_mode

    def system_voltage_status(self):
        voltage = min = self._sync_read(self.drive_space.ID(), DynamixelX.PRESENT_INPUT_VOLTAGE)


    @staticmethod
    def func_retry(func, args=None, retry_max=DynamixelX.RETRY_MAX, success_condition=True):
        result = None
        for retry_count in range(retry_max):
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
                logging.debug(f"Connection try {retry_count} succeed")
                break
        return result

    @staticmethod
    def create_drive_space(IDs, *args, **kwargs):
        d = DEF.define(prefix=ID, num=IDs, dtype=motor)
        return DefDict(d, name='dynamixel')

    @staticmethod
    def create_sense_space(IDs, *args, **kwargs):
        d = DEF.define(prefix=ID, num=IDs, dtype=sensor)
        return DefDict(d, name='dynamixel')

