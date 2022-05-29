import ray, time
import numpy as np
from sim.robots import RobotBase
from sim.utils.tictoc import tictoc
ROUND = 2

@ray.remote(num_cpus=1)
class SimActor:
    def __init__(self, robot, q_in, q_out):
        self.robot = robot
        self.q_in = q_in
        self.q_out = q_out

    def init(self):
        self.robot.init()

    def reset(self):
        self.robot.reset()

    def main(self):
        while True:
            data_in = self.q_in.get()
            data_out = self.step(*data_in)
            self.q_out.put_nowait(data_out)

    def drive(self, inpt, timestamp):
        self.inpt.data = inpt
        self.robot.drive(inpt, timestamp)

    def sense(self):
        return self.robot.sense()

    def observe_state(self):
        return self.robot.observe_state()

    def step(self, inpt, t_init, DT):
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
        dt_actual = time.perf_counter() -st
        return observe.data.as_list(), state.data.as_list(), info.data.as_list(), dt_actual

    def close(self):
        self.robot.close()
