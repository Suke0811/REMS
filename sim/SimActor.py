import ray, time
import numpy as np
from sim.robots import RobotBase
from sim.utils.tictoc import tictoc
ROUND = 2

@ray.remote
class SimActor:
    def __init__(self, robot):
        self.robot = robot

    def init(self):
        self.robot.init()

    def ret_robot(self):
        return self.robot

    def reset(self, inpt, t):
        self.robot.reset(inpt, t)

    def drive(self, inpt, timestamp):
        self.inpt.data = inpt
        self.robot.drive(inpt, timestamp)

    def sense(self):
        return self.robot.sense()

    def observe_state(self):
        return self.robot.observe_state()

    def step(self, inpt, t_init, DT):
        raise NotImplemented
        st = time.perf_counter()
        t = t_init
        state = None
        observe = None
        info = None
        while np.round(t - t_init, ROUND) < DT:
            self.robot.drive(inpt, t)
            observe = self.robot.sense()
            state = self.robot.observe_state()
            t = self.robot.clock(t)
            info = self.robot.info
        dt_actual = time.perf_counter() - st
        return observe.data.as_list(), state.data.as_list(), info.data.as_list(), dt_actual

    def step_forward(self, inpt, t_init, DT):
        st = time.perf_counter()
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
        dt_actual = time.perf_counter() - st
        return observe.data.as_list(), state.data.as_list(), info.data.as_list(), dt_actual

    def set_DT(self, DT):
        if self.robot.run.DT is None:
            self.robot.run.DT = DT

    def close(self):
        self.robot.close()
