import time

from sim.robots.RobotBase import RobotBase
from sim.bind.Dynamixel.Dynamixel import Dynamixel
from sim.typing import BindRule as rule
from sim.robots.scaler_leg.kinematics.wrap_to_pi import *
import numpy as np
import ray


class ScalerHard(RobotBase):
    ID_LISTs = []
    ID_LIST_SLAVEs = []
    ID_LISTs.append([1, 2, 3, 13, 14, 15, ])
    ID_LIST_SLAVEs.append([101, 102, 103])
    ID_LISTs.append([4, 5, 6, 16, 17, 18, ])
    ID_LIST_SLAVEs.append([104, 105, 106])
    ID_LISTs.append([7, 8, 9, 19, 20, 21,])
    ID_LIST_SLAVEs.append([107, 108, 109])
    ID_LISTs.append([10, 11, 12, 22, 23, 24,])
    ID_LIST_SLAVEs.append([110, 111, 112])

    ZERO_OFFSET = np.array([1.0 for _ in ID_LISTs[0]]) * np.pi

    DIRs = []
    OFFSETs = []
    DIRs.append(np.array([1, -1, 1, 1, 1, 1]))
    OFFSETs.append(np.array([0, np.pi / 2, -np.pi / 2, 0, 0, 0]) + ZERO_OFFSET)
    DIRs.append(np.array([1, 1, -1, 1, -1, 1]))
    OFFSETs.append(np.array([0, np.pi / 2, -np.pi / 2, 0, 0, np.pi]) + ZERO_OFFSET)
    DIRs.append(np.array([1, -1, 1, 1, 1, 1]))
    OFFSETs.append(np.array([0, np.pi / 2, -np.pi / 2, 0, 0, 0]) + ZERO_OFFSET)
    DIRs.append(np.array([1, 1, -1, 1, -1, 1]))
    OFFSETs.append(np.array([0, np.pi / 2, -np.pi / 2, 0, 0, np.pi]) + ZERO_OFFSET)


    def __init__(self, dynamiex_port, arm_id=3, *args, **kwargs):
        """init with a specific initial stat) """
        super().__init__(*args, **kwargs)
        self.dynamiex_port = dynamiex_port
        self.run.name = "Hard"
        self.run.to_thread = False
        self.sense_ref = []
        self.arm_id = arm_id
        self.which_leg = arm_id



    def init(self, init_state=None):
        """
        Initialization necessary for the robot. call all binded objects' init
        """
        # TODO: code clean up
        self.ID_LIST = self.ID_LISTs[self.arm_id]
        self.ID_LIST_SLAVE = self.ID_LIST_SLAVEs[self.arm_id]
        self.OFFSET = self.OFFSETs[self.arm_id]
        self.DIR = self.DIRs[self.arm_id]

        # binding rule
        # joint and main ids are positional match
        self.JOINT_ID_BIND = rule(self.joint_space.list_keys(), None, self.ID_LIST)
        # Hardware offset rule (from frame to hardware value)
        self.frame2hard = rule(self.joint_space.list_keys(),
                               lambda *vals: wrap_to_2pi((np.array(vals)+self.OFFSET) * self.DIR))
        # Hardware offset (inverse of frame2hard)
        self.dynamiexl_actor = DynamiexlActor.remote(self)
        ray.get(self.dynamiexl_actor.init.remote())
        motor = ray.get(self.dynamiexl_actor.get.remote('dynamixel', 'motors_outpt'))
        self.hard2frame = rule(motor.list_keys()[0:6],
                               lambda *vals: wrap_to_pi(np.array(vals) * self.DIR - self.OFFSET))


    def reset(self, inpt=None, t=None):
        if inpt is not None:
            self.drive(inpt, t)


    def drive(self, inpt, timestamp):
        # TODO: implement auto binding mechanism to remove this part
        self.inpt.set(inpt)
        self.joint_space.set(self.ik(inpt))
        joint = self.frame2hard.bind(self.joint_space)
        self.dynamiexl_actor.drive.remote(joint)

    def sense(self):
        # TODO: speedup this things
        if self.sense_ref:
            finished, self.sense_ref = ray.wait(self.sense_ref, num_returns=len(self.sense_ref))
            if finished:
                ret = ray.get(finished[-1])
                self.outpt.j().set(self.hard2frame.bind(ret.pos()))
                self.outpt.d_j().set(ret.vel().list())
        self.sense_ref.append(self.dynamiexl_actor.sense.remote())
        return self.outpt

    def observe_state(self):
        state = self.state.list()
        self.state.set(self.fk(self.outpt))
        self.calc_vel(pre_state=state, curr_state=self.state.list())
        return self.state

    def close(self):
        self.dynamiexl_actor.close.remote()

    def __del__(self):
        return

    def print(self):
        print('hi')


@ray.remote#(num_cpus=1)#(max_restarts=5, max_task_retries=-1)
class DynamiexlActor:
    def __init__(self, data):
        self.data = data
        self.quite = False
        self.block = True

    def get(self, *name):
        attr = self
        for n in name:
            attr = getattr(attr, n)
        return attr

    def init(self):
        self.dynamixel = Dynamixel(self.data.ID_LIST, self.data.ID_LIST_SLAVE, self.data.dynamiex_port)
        self.dynamixel.init()
        return True

    def drive(self, inpt):
        # TODO: implement auto binding mechanism to remove this part
        dynamixel_inpt = self.dynamixel.motors_inpt
        dynamixel_inpt.pos().set(inpt)
        self.dynamixel.drive(dynamixel_inpt, 0)

    def sense(self):
        return self.dynamixel.sense()

    def close(self):
        self.dynamixel.close()
        return False

