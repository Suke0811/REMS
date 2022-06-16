import ray, time
import numpy as np
from sim.sim_handler.ray.autocounter import auto_count
from sim.robots import RobotBase
from sim.utils.tictoc import tictoc
import logging
ROUND = 2


@ray.remote
class SimActor:
    def __init__(self, robot, outputs):
        self.robot = robot
        self.outputs = outputs
        logging.getLogger().setLevel(logging.INFO)

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
        return observe, state, info, dt_actual, t

    def step(self, inpt, t_init, DT):
        outpt, state, info, dt_actual, t = self.step_forward(inpt, t_init, DT)
        for out in self.outputs:
            out.process(state, inpt, outpt, t, info)

        #if not self.suppress_info:
        logging.info("Name: {}, dt: {}, t: {}, inpt: {}, state: {}, output: {}, info: {}".format(
            self.robot.run.name,
            np.round(dt_actual, 5), np.round(t, ROUND),
            {k: round(v, ROUND) for k, v in inpt.items()},
            {k: round(v, ROUND) for k, v in state.items()},
            {k: round(v, ROUND) for k, v in outpt.items()},
            info))


    def set_DT(self, DT):
        if self.robot.run.DT is None:
            self.robot.run.DT = DT

    def _call_func(self, name, *args, **kwargs):
        return getattr(self.robot, name)(*args, **kwargs)
    # for communication
    def _get_variable(self, name):
        return getattr(self.robot, name)

    def _set_variable(self, name, val):
        #eval("self.robot."+name+"= val")
        setattr(self.robot, name, val)
        #getattr(self.robot, name)

    def change_val(self,val):
        self.robot.run.DT = val


