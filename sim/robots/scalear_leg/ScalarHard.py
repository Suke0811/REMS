import time

import sim.type.definitions
from sim.robots.RobotBase import RobotBase
from sim.robots.bind.Dynamixel.Dynamixel import Dynamixel
from sim.type import DefBindRule as rule
from sim.robots.scalear_leg.kinematics.wrap_to_pi import wrap_to_2pi
from sim.type.definitions import *
import numpy as np
import logging
from sim.utils.tictoc import tictoc
import ray
from ray.util.queue import Queue, Empty


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
        self.run.name = "Hard"
        self.run.to_thread = False
        self.sense_ref = None

    def init(self, init_state=None):
        """Initialization necessary for the robot. call all binded objects' init
                """
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
        #self.hard2frame = rule(self.dynamixel.motor_pos.DEF.key_as_list(),
         #                           lambda *vals: wrap_to_2pi(np.array(vals) * self.DIR - self.OFFSET))


        self.dynamiexl_actor = DynamiexlActor.options(name="dynamixel").remote(self)
        ray.get(self.dynamiexl_actor.init.remote())
        #self.dynamiexl_actor.main_loop.remote()


    def drive(self, inpt, timestamp):
        # TODO: implement auto binding mechanism to remove this part
        self.inpt.data = inpt
        #self.queue.put(inpt)
        self.dynamiexl_actor.drive.remote(inpt)


    def sense(self):
        ready = []
        # Allow 1000 in flight calls
        # For example, if i = 5000, this call blocks until that
        # 4000 of the object_refs in result_refs are ready
        # and available.
        # if len(self.ray_refs) > 10:
        #     num_ready = len(self.ray_refs) - 10
        #     ready, self.ray_refs = ray.wait(self.ray_refs, num_returns=num_ready)
        # self.ray_refs.append(self.dynamiexl_actor.sense.remote())
        # if ready:
        #     self.outpt.data = ray.get(ready[-1])
        if self.sense_ref is not None:
            self.outpt.data = ray.get(self.sense_ref)
        self.sense_ref = self.dynamiexl_actor.sense.remote()
        return self.outpt

    def observe_state( self):
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

    def __del__(self):
        ray.kill(self.dynamiexl_actor)



@ray.remote(num_cpus=1)#(max_restarts=5, max_task_retries=-1)
class DynamiexlActor:
    def __init__(self, data):
        self.data = data
        self.quite = False
        self.block = True
        #self.queue_in = data.queue
        # self.data_in = self.data.inpt

    def init(self):
        self.dynamixel = Dynamixel(self.data.ID_LIST, self.data.ID_LIST_SLAVE, self.data.dynamiex_port)
        self.dynamixel.init()

    # def main_loop(self):
    #     while not self.quite:
    #         self.data_in.data = self.receive_data()
    #         self.drive(self.data_in)

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
        return self.data.outpt

    # def send_data(self):    # sending data won't block the process
    #     self.queue_out.put_nowait(self.data_out.data_as([self.robot, self.out]))

    # def receive_data(self):
    #     """Receiving data can block the process if necessary"""
    #     try:
    #         self.data_in = self.queue_in.get(self.block)
    #     except Empty:
    #         pass

    def __del__(self):
        self.dynamixel.close()



if __name__ == '__main__':
    from sim.robots.bind_robot import bind_robot
    from sim.robots.scalear_leg.ScalerManipulatorDef import ScalerManipulator

    d = bind_robot(ScalerManipulator, ScalerHard, 'COM5')
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


