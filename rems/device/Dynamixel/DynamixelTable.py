from .dynamixel_sdk.robotis_def import *

import numpy as np

class TableBase:
    """
    Base class for Dynamixel control table
    :param ADDR: address of the control parameters
    :param LEN: length of the control parameters
    :param R: if readable
    :param W: if writable
    :param RANGE: value range
    :param UNIT: unit (from unit to tic)
    """
    ADDR = 0
    LEN = 1
    R = True
    W = True
    RANGE = None
    UNIT = 1.0

    @classmethod
    def from_unit(cls, val):
        """
        Convert unitful values to 4 digits HEX
        :param val: value to convert
        :return: 4 digit HEX in list
        """
        v = int(val / cls.UNIT)
        # Allocate goal position value into byte array

        # A word = 4 digits HEX number = 16 bits binary number = 0 - 65535, thus 65536 has loword=0 and hiword=1
        # LOBYTE and HIBYTE separates low 4 digit HEX into 2 low + 2 high digits and express in DEX
        # e.g. 2047 has LOBYTE=255 and HIBYTE=7, since 15 + 15*16 = 255, 255 + 7*16^2 = 2047
        byte = [DXL_LOBYTE(DXL_LOWORD(v)), DXL_HIBYTE(DXL_LOWORD(v)),
                DXL_LOBYTE(DXL_HIWORD(v)), DXL_HIBYTE(DXL_HIWORD(v))]
        return byte[0:cls.LEN]

    @classmethod
    def to_unit(cls, val):
        """
        Convert dynamixel tic to unitful values
        :param val: dynamixel tic (not HEX)
        :return: value in corresponding unit
        """
        return val * cls.UNIT

class classproperty(object):
    def __init__(self, fget):
        self.fget = fget

    def __get__(self, owner_self, owner_cls):
        return self.fget(owner_cls)

# raise not implemented error when a class field is accessed
@classproperty
def NotImplementedField(self):
    raise NotImplementedError('Table does not exists')

# a table to throw NotImplementedError
class NotAavailableTable(TableBase):
    ADDR = NotImplementedField
    LEN = NotImplementedField
    R = False
    W = False
    RANGE = NotImplementedField
    UNIT = NotImplementedField


class unit:
    rad = 0.00153398
    rad_p_sec = 0.02398
    percentage = 0.00113
    amps = 0.00269
    accel = 0.3745
    msec = 0.001

    percentage_pro = 0.0498/100
    rad_p_sec_pro = 0.001047
    amps_pro = 0.001



class DynamixelX:
    """
    Control tables for Dynamixel X series.
    (Pro or legacy lines have different addresses)
    """
    PROTOCOL_VERSION = 2.0
    BAUDRATE = 4000000
    RETRY_MAX = 5       # # of times to retry communication
    class TORQUE_ENABLE(TableBase):
        ADDR = 64

    class LED(TableBase):
        ADDR = 65

    class STATUS_RETURN_LEVEL(TableBase):
        ADDR = 68
        LEN = 1

    class REGISTERED_INSTRUCTION(TableBase):
        ADDR = 69
        LEN = 1
        W = False

    class HARDWARE_ERROR_STATUS(TableBase):
        ADDR = 70
        LEN = 1
        W = False

    class VELOCITY_I_GAIN(TableBase):
        ADDR = 76
        LEN = 2

    class VELOCITY_P_GAIN(TableBase):
        ADDR = 78
        LEN = 2

    class POSITION_D_GAIN(TableBase):
        ADDR = 80
        LEN = 2

    class POSITION_I_GAIN(TableBase):
        ADDR = 82
        LEN = 2

    class POSITION_P_GAIN(TableBase):
        ADDR = 84
        LEN = 2

    class FEEDFORWARD_2ND_GAIN(TableBase):
        ADDR = 88
        LEN = 2

    class FEEDFORWARD_1ST_GAIN(TableBase):
        ADDR = 90
        LEN = 2

    class BUS_WATCHDOG(TableBase):
        ADDR = 98
        LEN = 1

    class GOAL_PWM(TableBase):
        ADDR = 100
        LEN = 2
        UNIT = unit.percentage

    class GOAL_CURRENT(TableBase):
        ADDR = 102
        LEN = 2
        UNIT = unit.amps    # amps

    class GOAL_VELOCITY(TableBase):
        ADDR = 104
        LEN = 4
        UNIT = unit.rad_p_sec  # rad/sec

    class PROFILE_ACCELERATION(TableBase):
        ADDR = 108
        LEN = 4
        UNIT = unit.accel

    class PROFILE_VELOCITY(TableBase):
        ADDR = 112
        LEN = 4
        UNIT = unit.rad_p_sec   # rad/sec

    class GOAL_POSITION(TableBase):
        ADDR = 116
        LEN = 4
        UNIT = unit.rad  # rad

    class REALTIME_CLICK(TableBase):
        ADDR = 120
        LEN = 2
        UNIT = unit.msec # msec
        W = False

    class MOVING(TableBase):
        ADDR = 122
        LEN = 1
        W = False

    class MOVING_STATUS(TableBase):
        ADDR = 123
        LEN = 1
        W = False

    class PRESENT_PWM(TableBase):
        ADDR = 124
        LEN = 2
        W = False

    class PRESENT_CURRENT(TableBase):
        ADDR = 126
        LEN = 2
        W = False
        UNIT = unit.amps #amps

    class PRESENT_VELOCITY(TableBase):
        ADDR = 128
        LEN = 4
        W = False
        @staticmethod
        def to_unit(val):
            """
            Convert raw motor velocity values [0, 32767) to radian / sec
            :param val: motor velocity
            :return: rad/sec: radian per second.
            """
            # check two's complement
            if val > 0x7fffffff:
                val -= 4294967296

            rev_min = val * 0.442
            rad_sec = (rev_min * 2 * np.pi) / 60
            return rad_sec

    class PRESENT_POSITION(TableBase):
        ADDR = 132
        LEN = 4
        W = False
        UNIT = unit.rad     # rad

    class VELOCITY_TRAJECTORY(TableBase):
        ADDR = 136
        LEN = 4
        W = False
        UNIT = unit.rad_p_sec

    class POSITION_TRAJECTORY(TableBase):
        ADDR = 140
        LEN = 4
        W = False
        UNIT = unit.rad

    class PRESENT_INPUT_VOLTAGE(TableBase):
        ADDR = 144
        LEN = 2
        W = False
        UNIT = 0.1  #volts

    class PRESENT_TEMPERATURE(TableBase):
        ADDR = 146
        LEN = 1
        W = False

    class BACKUP_READY(TableBase):
        ADDR = 147
        LEN = 1
        W = False


    ## EEPROM
    class OPERATING_MODE(TableBase):
        ADDR = 11
        LEN = 1
        VEL_MODE = 1
        POS_MODE = 3

class DynamixelLegacy(DynamixelX):
    """
    Control tables for Dynamixel legacy series.
    The same as DynamiexX
    147 Not Available
    """

    class BACKUP_READY(NotAavailableTable):
        pass
    pass

class DynamixelPro:
    """
    Control tables for Dynamixel Pro series.
    """
    PROTOCOL_VERSION = 2.0
    BAUDRATE = 4000000
    RETRY_MAX = 5       # # of times to retry communication
    class TORQUE_ENABLE(TableBase):
        ADDR = 512

    class LED_RED(TableBase):
        ADDR = 513

    class LED_GREEN(TableBase):
        ADDR = 514

    class LED_BLUE(TableBase):
        ADDR = 515

    class STATUS_RETURN_LEVEL(TableBase):
        ADDR = 516
        LEN = 1

    class REGISTERED_INSTRUCTION(TableBase):
        ADDR = 517
        LEN = 1
        W = False

    class HARDWARE_ERROR_STATUS(TableBase):
        ADDR = 518
        LEN = 1
        W = False

    class VELOCITY_I_GAIN(TableBase):
        ADDR = 524
        LEN = 2

    class VELOCITY_P_GAIN(TableBase):
        ADDR = 526
        LEN = 2

    class POSITION_D_GAIN(TableBase):
        ADDR = 528
        LEN = 2

    class POSITION_I_GAIN(TableBase):
        ADDR = 530
        LEN = 2

    class POSITION_P_GAIN(TableBase):
        ADDR = 532
        LEN = 2

    class FEEDFORWARD_2ND_GAIN(TableBase):
        ADDR = 536
        LEN = 2

    class FEEDFORWARD_1ST_GAIN(TableBase):
        ADDR = 538
        LEN = 2

    class BUS_WATCHDOG(TableBase):
        ADDR = 546
        LEN = 1

    class GOAL_PWM(TableBase):
        ADDR = 548
        LEN = 2
        UNIT = unit.percentage_pro

    class GOAL_CURRENT(TableBase):
        ADDR = 550
        LEN = 2
        UNIT = unit.amps_pro    # amps

    class GOAL_VELOCITY(TableBase):
        ADDR = 552
        LEN = 4
        UNIT = unit.rad_p_sec_pro  # rad/sec

    class PROFILE_ACCELERATION(TableBase):
        ADDR = 556
        LEN = 4
        UNIT = unit.accel

    class PROFILE_VELOCITY(TableBase):
        ADDR = 560
        LEN = 4
        UNIT = unit.rad_p_sec_pro   # rad/sec

    class GOAL_POSITION(TableBase):
        ADDR = 564
        LEN = 4
        UNIT = unit.rad  # rad

    class REALTIME_CLICK(TableBase):
        ADDR = 568
        LEN = 2
        UNIT = unit.msec  # msec
        W = False

    class MOVING(TableBase):
        ADDR = 570
        LEN = 1
        W = False

    class MOVING_STATUS(TableBase):
        ADDR = 571
        LEN = 1
        W = False

    class PRESENT_PWM(TableBase):
        ADDR = 572
        LEN = 2
        W = False

    class PRESENT_CURRENT(TableBase):
        ADDR = 574
        LEN = 2
        W = False
        UNIT = unit.amps #amps

    class PRESENT_VELOCITY(TableBase):
        ADDR = 576
        LEN = 4
        W = False
        UNIT = unit.rad_p_sec_pro
        @staticmethod
        def to_unit(val):
            """
            Convert raw motor velocity values [0, 32767) to radian / sec
            :param val: motor velocity
            :return: rad/sec: radian per second.
            """
            # check two's complement
            if val > 0x7fffffff:
                val -= 4294967296

            rev_min = val * 0.442
            rad_sec = (rev_min * 2 * np.pi) / 60
            return rad_sec

    class PRESENT_POSITION(TableBase):
        ADDR = 580
        LEN = 4
        W = False
        UNIT = unit.rad     # rad

    class VELOCITY_TRAJECTORY(TableBase):
        ADDR = 584
        LEN = 4
        W = False
        UNIT = unit.rad_p_sec_pro

    class POSITION_TRAJECTORY(TableBase):
        ADDR = 588
        LEN = 4
        W = False
        UNIT = unit.rad

    class PRESENT_INPUT_VOLTAGE(TableBase):
        ADDR = 592
        LEN = 2
        W = False
        UNIT = 0.1  #volts

    class PRESENT_TEMPERATURE(TableBase):
        ADDR = 594
        LEN = 1
        W = False
