import dynamixel_sdk as x
from dynamixel_sdk.robotis_def import *

class TableBase:
    ADDR = 0
    LEN = 1
    R = True
    W = True
    RANGE = None
    UNIT = 1.0

    @classmethod
    def from_unit(cls, val):
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
        return val * cls.UNIT



class XM430:
    RETRY_MAX = 5
    PROTOCOL_VERSION = 2.0
    BAUDRATE = 4000000
    class TORQUE_ENABLE(TableBase):
        ADDR = 64

    class LED(TableBase):
        ADDR = 65


    class GOAL_POSITION(TableBase):
        ADDR = 116
        LEN = 4
        UNIT = 0.00153398  # rad

    class GOAL_VELOCITY(TableBase):
        ADDR = 112
        LEN = 4
        UNIT = 0.02398  # rev/sec

    class PRESENT_POSITION(TableBase):
        ADDR = 132
        LEN = 4
        W = False
        UNIT = 0.00153398 # rad

    class PRESENT_VELOCITY(TableBase):
        ADDR = 128
        LEN = 4
        W = False
        UNIT = 0.02398 # rev/sec

    class OPERATING_MODE(TableBase):
        ADDR = 11
        LEN = 1
        VEL_MODE = 1
        POS_MODE = 3





pass

