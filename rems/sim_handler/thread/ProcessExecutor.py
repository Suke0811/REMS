import time

from rems.device.DeviceBase import DeviceBase
from rems.sim_handler.ray.RayWrapper import RayWrapper
from time import perf_counter
from rems.typing import DefDict
import copy
from threading import Thread, Lock
from rems.utils import tictoc
from rems.process import ProcessSystem


class ProcessExecutor(ProcessSystem):
    def __init__(self, process, debug_mode=False, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pro = process
        self.debug_mode = debug_mode
        self.t_current = 0.0
        self.dt = process.dt
        self.threading = process.to_thread and not debug_mode
        self.lock = Lock()
        self.timestep = 0.0
        self.run_process = False

    def start(self, realtime=False):
        if realtime:
            self.t_current = perf_counter()
        while self.threading:   # if threading is false, then this job simply dies
            if self.if_time():
                with self.lock:
                    self.pro.process(self.timestep, block=False)
                    self.run_process = False
            time.sleep(0.0001)

    def init(self, robot=None, *args, **kwargs):
        self.pro.init(robot, *args, **kwargs)

    def if_time(self):
        return self.run_process
        if self.timestep >= self.t_current + self.dt:
            self.t_current += self.dt
            return True

        # if perf_counter() >= self.t_current + self.dt:
        #     self.t_current += self.dt
        #     return True
        return False

    def process(self, t, *args, **kwargs):
        # if t == 0.0:
        #     return
        if self.threading:
            self.timestep = t
            self.run_process = True
        else:
            self.pro.process(t, *args, **kwargs)
