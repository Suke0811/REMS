from dynamixel_sdk.robotis_def import *
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

class unit:
    rad = 0.00153398
    rad_p_sec = 0.02398



class DynamiexX:
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

    class GOAL_POSITION(TableBase):
        ADDR = 116
        LEN = 4
        UNIT = unit.rad  # rad

    class GOAL_VELOCITY(TableBase):
        ADDR = 104
        LEN = 4
        UNIT = unit.rad_p_sec  # rev/sec


    class PROFILE_VELOCITY(TableBase):
        ADDR = 112
        LEN = 4
        UNIT = unit.rad_p_sec   # rev/sec

    class PROFILE_ACCELERATION(TableBase):
        ADDR = 108
        LEN = 4
        UNIT = 0.3745

    class PRESENT_POSITION(TableBase):
        ADDR = 132
        LEN = 4
        W = False
        UNIT = unit.rad # rad

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

    class OPERATING_MODE(TableBase):
        ADDR = 11
        LEN = 1
        VEL_MODE = 1
        POS_MODE = 3
