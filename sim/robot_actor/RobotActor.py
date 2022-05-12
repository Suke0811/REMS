import ray
import time
import asyncio
from ray.util.queue import Queue
from sim.robot_actor.definition_queue import *

@ray.remote(num_cpus=1)
class RobotActor:
    def __init__(self, robot, out, queue_in: Queue, queue_out: Queue, wait_for_signal=True):
        self.robot = robot
        self.out = out
        self.queue_in = queue_in
        self.queue_out = queue_out
        self.data_in = DefDict(QUEUE_IN)
        self.data_out = DefDict(QUEUE_OUT)
        self.block = wait_for_signal

    def init(self):
        self.robot.init()

    def reset(self):
        self.robot.reset()

    def main_loop(self):
        while not self.quite:
            self.receive_data()
            self.step(*self.data_in.data())
            self.send_data()

    def step(self, inpt: DefDict, t_init, dt):
        t = t_init
        state = None
        observe = None
        info = None
        while t - t_init < dt:
            state = self.robot.drive(inpt, t)
            observe = self.robot.sense()
            t = self.robot.clock(t)
            info = self.robot.get_update()
        for out in self.out:
            out.process(state, inpt, observe, t, info)
            


    def send_data(self):
        self.queue_out.put_nowait(self.data_out.data_as([self.robot, self.out]))

    def receive_data(self):
        self.data_in.data = self.queue_in.get(self.block)




