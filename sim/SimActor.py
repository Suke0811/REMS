import ray, time
import numpy as np
from sim.robots import RobotBase
from sim.utils.tictoc import tictoc
ROUND = 2

@ray.remote(num_cpus=1)
class SimActor:
    def __init__(self, robot):
        self.robot = robot

    def init(self):
        self.robot.init()

    def reset(self):
        self.robot.reset()

    def drive(self, inpt, timestamp):
        self.robot.drive(inpt, timestamp)

    def sense(self):
        return self.robot.sense()

    def observe_state(self):
        return self.robot.observe_state()

    def step_forward(self, inpt, t_init, DT):
        t=t_init
        state = None
        observe = None
        info = None
        while np.round(t - t_init, ROUND) < DT:
            self.robot.drive(inpt, t)
            observe = self.robot.sense()
            state = self.robot.observe_state()

            t = self.robot.clock(t)
            info = self.robot.info
        return observe, state, info
