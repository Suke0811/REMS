from sim.robots.RobotBase import RobotBase
from sim.robots.bind.Dynamixel.Dynamixel import Dynamixel
from sim.type import DefBindRule as rule
from sim.robots.scalear_leg.kinematics.wrap_to_pi import wrap_to_2pi
from sim.type.definitions import *
import numpy as np




class ScalerHard(RobotBase):
    ID_LIST =[str(n) for n in [10,11,12,22,23,24]]
    ID_LIST_SLAVE =[str(n) for n in [110, 111, 112]]

    SLAVE_PREFIX = '1'

    HOME_POSITION = [0.0 for _ in ID_LIST]
    DIR = np.array([1, 1, -1, 1, -1, 1])
    ZERO_OFFSET = np.array([1.0 for _ in ID_LIST]) * np.pi
    OFFSET = np.array([0, np.pi / 2, -np.pi / 2, 0, 0, 0]) + ZERO_OFFSET


    def __init__(self, dynamiex_port, *args, **kwargs):
        """init with a specific initial stat) """
        super().__init__(*args, **kwargs)
        self.dynamiex_port = dynamiex_port

    def init(self, init_state=None):
        """Initialization necessary for the robot. call all binded objects' init
                """
        # slave joints are coupled to certain servos
        self.JOINT_SLAVE_ID_BIND = rule(self.ID_LIST, lambda *vals: [i for i in vals], self.ID_LIST_SLAVE)
        self.dynamixel = Dynamixel(self.ID_LIST, self.ID_LIST_SLAVE, self.dynamiex_port)
        # binding rule
        # joint and main ids are positional match
        self.JOINT_ID_BIND = rule(self.joint_space.DEF.key_as_list(), None, self.ID_LIST)
        # Hardware offset rule (from frame to hardware value)
        self.frame2hard = rule(self.joint_space.DEF.key_as_list(),
                                    lambda *vals: wrap_to_2pi((np.array(vals)+self.OFFSET) * self.DIR))
        # Hardware offset (inverse of frame2hard)
        self.hard2frame = rule(self.dynamixel.motor_pos.DEF.key_as_list(),
                                    lambda *vals: wrap_to_2pi(np.array(vals) * self.DIR - self.OFFSET))

        self.dynamixel.init()

    def drive(self, inpt, timestamp):
        # TODO: implement auto binding mechanism to remove this part
        self.inpt = inpt
        state = self.state
        dynamixel_inpt = self.dynamixel.motors
        # TODO: binding for offsets
        joint = self.frame2hard.bind(self.ik(inpt))
        self.joint_space.data = joint
        dynamixel_inpt.data = joint
        dynamixel_inpt.data = self.JOINT_SLAVE_ID_BIND.bind(dynamixel_inpt)
        self.dynamixel.drive(dynamixel_inpt, timestamp)



    def sense(self):
        s = self.dynamixel.sense()
        s.data = self.hard2frame.bind(s)
        self.outpt.data = s.data.as_list()
        return self.outpt

    def observe_state(self):
        state = self.state
        self.state.set_data(self.fk(self.outpt))
        self.calc_vel(pre_state=state, curr_state=self.state)
        return self.state

    def calc_vel(self, pre_state, curr_state):
        prev_state = pre_state.data_as(POS_3D).data.as_list()
        self.state.data = self.task_space
        next_state = curr_state.data_as(POS_3D).data.as_list()
        dx = (next_state[0] - prev_state[0]) / self.run.DT
        dy = (next_state[1] - prev_state[1]) / self.run.DT
        dz = (next_state[2] - prev_state[2]) / self.run.DT
        self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}
