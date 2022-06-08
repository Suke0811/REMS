import time

from sim.robots.RobotBase import RobotBase
from sim.bind.Dynamixel.Dynamixel import Dynamixel
from sim.type import DefBindRule as rule
from sim.robots.scalear_leg.kinematics.wrap_to_pi import *
import numpy as np
import ray


class ScalerHard(RobotBase):
    ID_LISTs = []
    ID_LIST_SLAVEs = []
    ID_LISTs.append([str(n) for n in [1, 2, 3, 13, 14, 15, ]])
    ID_LIST_SLAVEs.append([str(n) for n in [101, 102, 103]])
    ID_LISTs.append([str(n) for n in [4, 5, 6, 16, 17, 18, ]])
    ID_LIST_SLAVEs.append([str(n) for n in [104, 105, 106]])
    ID_LISTs.append([str(n) for n in [7, 8, 9, 19, 20, 21, ]])
    ID_LIST_SLAVEs.append([str(n) for n in [107, 108, 109]])
    ID_LISTs.append([str(n) for n in [10, 11, 12, 22, 23, 24, ]])
    ID_LIST_SLAVEs.append([str(n) for n in [110, 111, 112]])

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
        # slave joints are coupled to certain servos
        self.JOINT_SLAVE_ID_BIND = rule(self.ID_LIST, lambda *vals: [i for i in vals], self.ID_LIST_SLAVE)
        #self.dynamixel = Dynamixel(self.ID_LIST, self.ID_LIST_SLAVE, self.dynamiex_port)
        # binding rule
        # joint and main ids are positional match
        self.JOINT_ID_BIND = rule(self.joint_space.DEF.key_as_list(), None, self.ID_LIST)
        # Hardware offset rule (from frame to hardware value)
        self.frame2hard = rule(self.joint_space.DEF.key_as_list(),
                               lambda *vals: wrap_to_2pi((np.array(vals)+self.OFFSET) * self.DIR))
        # Hardware offset (inverse of frame2hard)
        self.dynamiexl_actor = DynamiexlActor.remote(self)
        ray.get(self.dynamiexl_actor.init.remote())


    def reset(self, inpt=None, t=None):
        if inpt is not None:
            self.drive(inpt, t)


    def drive(self, inpt, timestamp):
        # TODO: implement auto binding mechanism to remove this part
        self.inpt.data = inpt
        self.dynamiexl_actor.drive.remote(inpt)

    def sense(self):
        # TODO: speedup this things
        if self.sense_ref:
            finished, self.sense_ref = ray.wait(self.sense_ref, num_returns=len(self.sense_ref))
            if finished:
                self.outpt.data = ray.get(finished[-1])
        self.sense_ref.append(self.dynamiexl_actor.sense.remote())
        return self.outpt

    def observe_state(self):
        state = self.state.data.as_list()
        self.state.set_data(self.fk(self.outpt))
        self.calc_vel(pre_state=state, curr_state=self.state.data.as_list())
        return self.state

    def calc_vel(self, pre_state, curr_state):
        dx = (curr_state[0] - pre_state[0]) / self.run.DT
        dy = (curr_state[1] - pre_state[1]) / self.run.DT
        dz = (curr_state[2] - pre_state[2]) / self.run.DT
        self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}

    def close(self):
        self.dynamiexl_actor.close.remote()

    def __del__(self):
        return


@ray.remote#(num_cpus=1)#(max_restarts=5, max_task_retries=-1)
class DynamiexlActor:
    def __init__(self, data):
        self.data = data
        self.quite = False
        self.block = True
        #self.queue_in = data.queue
        # self.data_in = self.data.inpt

    def init(self):
        self.dynamixel = Dynamixel(self.data.ID_LIST, self.data.ID_LIST_SLAVE, self.data.dynamiex_port)
        self.hard2frame = rule(self.dynamixel.motor_pos.DEF.key_as_list(),
                               lambda *vals: wrap_to_pi(np.array(vals) * self.data.DIR - self.data.OFFSET))
        self.dynamixel.init()
        return True

    def main_loop(self):
        while not self.quite:
            self.data_in.data = self.receive_data()
            self.drive(self.data_in)

    def drive(self, inpt):
        # TODO: implement auto binding mechanism to remove this part
        self.data.inpt = inpt
        dynamixel_inpt = self.dynamixel.motors
        joint = self.data.frame2hard.bind(self.data.ik(inpt))
        self.data.joint_space.data = joint
        dynamixel_inpt.data = joint
        dynamixel_inpt.data = self.data.JOINT_SLAVE_ID_BIND.bind(dynamixel_inpt)
        self.dynamixel.drive(dynamixel_inpt, 0)

    def sense(self):
        s = self.dynamixel.sense()
        s.data = self.hard2frame.bind(s)
        self.data.outpt.data = s.data.as_list()
        return self.data.outpt#.data.as_list()

    def close(self):
        self.dynamixel.close()
        return False

if __name__ == '__main__':
    from sim.robots.bind_robot import bind_robot
    from sim.robots.scalear_leg.ScalerManipulatorDef import ScalerManipulator

    d = bind_robot(ScalerManipulator, ScalerHard, '/dev/ttyUSB0')
    d.init()
    d.reset()
    i = d.inpt

    i.data = [0,0,-0.350]
    c = True
    N = 10000
    for n in range(N):
        if n % 100 == 0:
            if c:
                i.data = [0,0, -0.25]
                c = False
            else:
                i.data = [0,0,-0.35]
                c = True
        d.drive(i, 0)
        #t = d.clock(n)
        time.sleep(0.1)
        print(n)
        #d.sense()


