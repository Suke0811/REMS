
class TableBase:
    ADDR = 0
    LEN = 1
    R = True
    W = True


class XM430:
    PROTOCOL_VERSION = 2.0
    BAUDRATE = 4000000
    class TORQUE_ENABLE(TableBase):
        ADDR = 64

    class GOAL_POSITION(TableBase):
        ADDR = 116
        LEN = 4

    class GOAL_VELOCITY(TableBase):
        ADDR = 112
        LEN = 4

    class PRESENT_POSITION(TableBase):
        ADDR = 132
        LEN = 4






