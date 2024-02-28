import logging
import numpy as np
import time, signal
from rems.sim_handler import ProcessActor
from rems.Config import SimConfig
from rems.robots.bind_robot import bind_robot
ROUND = 2


class Operator:
    DT_ERR = 0.01
    DT_DEFAULT = 0.25
    def __new__(cls, lite_operator=True, *args, **kwargs):
        if lite_operator:
            from rems.sim_handler.LiteOperator import LiteOperator
            return super(Operator, cls).__new__(LiteOperator)
        else:
            from rems.sim_handler.RayOperator import RayOperator
            return super(Operator, cls).__new__(RayOperator)

    def __init__(self, debug_mode=False, suppress_info=False, *args, **kwargs):
        self.suppress_info = suppress_info
        self._input_system = None
        self._robots = []
        self._processes = []
        self.realtime = False
        self.runconfig = None
        self.DT = None
        signal.signal(signal.SIGINT, self.handler_ctrl_c)
        self.made_outputs = False

    def handler_ctrl_c(self, signum, frame):
        """Ctrl+C termination handling"""
        self.close()
        self.make_outputs()
        exit(1)

    def set_input(self, input_system):
        """set InputSystem
        :param input_system: input to be used (child of InputSystem)"""
        input_system.init()
        self._input_system = input_system

    def add_robot(self, robot_def=None, robot=None, def_args=None, robot_args=None, outputs=None, inpt=None, *args, **kwargs):
        raise NotImplementedError("This method should be implemented in the subclass")

    def add_process(self, process, *args, **kwargs):
        raise NotImplementedError("This method should be implemented in the subclass")

    def init(self, t):
        raise NotImplementedError("This method should be implemented in the subclass")

    def init_process(self):
        raise NotImplementedError("This method should be implemented in the subclass")

    def open(self):
        raise NotImplementedError("This method should be implemented in the subclass")

    def reset(self, t):
        raise NotImplementedError("This method should be implemented in the subclass")

    def close(self):
        raise NotImplementedError("This method should be implemented in the subclass")

    def set_dt(self, DT):
        raise NotImplementedError("This method should be implemented in the subclass")

    def run(self, config: SimConfig):
        raise NotImplementedError("This method should be implemented in the subclass")

    def step(self, t):
        raise NotImplementedError("This method should be implemented in the subclass")

    def process(self, t):
        raise NotImplementedError("This method should be implemented in the subclass")

    def return_handle(self, ret, t, ret_robot):
        raise NotImplementedError("This method should be implemented in the subclass")

    def make_outputs(self):
        raise NotImplementedError("This method should be implemented in the subclass")
