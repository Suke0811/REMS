from sim.robots.RobotBase import RobotBase
from sim.robots.bind.Dynamixel.Dynamixel import Dynamixel
from sim.type import DefBindRule as rule
import numpy as np




class ScalerHard(RobotBase):
    ID_LIST =[str(n) for n in [10,11,12,22,23,24]]
    ID_LIST_SLAVE =[str(n) for n in [110, 111, 112]]
    SLAVE_PREFIX = '1'

    HOME_POSITION = [0.0 for _ in ID_LIST]
    DIR = np.array([1, 1, -1, 1, -1, 1])
    OFFSET = np.array([0, np.pi / 2, -np.pi / 2, 0, 0, 0])

    def __init__(self, dynamiex_port, *args, **kwargs):
        """init with a specific initial stat) """
        super().__init__(*args, **kwargs)
        self.dynamiex_port = dynamiex_port
        self.run.to_thread = False



    def init(self, init_state=None):
        """Initialization necessary for the robot. call all binded objects' init
        """
        # binding rule
        # joint and main ids are positional match
        self.JOINT_ID_BIND = rule(self.joint_space.DEF.as_list(), self.ID_LIST)
        # slave joints are coupled to certain servos
        self.JOINT_SLAVE_ID_BIND = rule(self.ID_LIST, lambda *ids: tuple(self.SLAVE_PREFIX+i for i in ids), self.ID_LIST_SLAVE)

        self.dynamixel = Dynamixel(self.ID_LIST.extend(self.ID_LIST_SLAVE), self.dynamiex_port)
        self.dynamixel.init()

    def drive(self, inpt, timestamp):
        # TODO: implement auto binding mechanism to remove this part
        dynamixel_inpt = self.dynamixel.motors
        # TODO: binding for offsets
        joint = self.ik(inpt).data.as_numpy() * self.DIR + self.OFFSET
        self.joint_space.data = joint
        dynamixel_inpt.data = self.JOINT_ID_BIND.bind(inpt)
        dynamixel_inpt.data = self.JOINT_SLAVE_ID_BIND.bind(dynamixel_inpt)

        self.dynamixel.drive(dynamixel_inpt, timestamp)

    def sense(self):
        s = self.dynamixel.sense()
        self.outpt.data = s.data.as_list()
