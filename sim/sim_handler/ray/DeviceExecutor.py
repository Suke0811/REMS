import threading
import time

from sim.device.DeviceBase import DeviceBase
from sim.sim_handler.ray.RayWrapper import RayWrapper
from time import perf_counter
from sim.typing import DefDict
import copy
from threading import Thread, Lock


class DeviceExecutor(DeviceBase):
    def __init__(self, device: DeviceBase, *args, **kwargs):
        self._inpt = None
        self._state = None
        self._outpt = None
        self._timestep = 0.0

        self.threading = False
        if device.to_thread == device.TO_PROCESS:
            device = RayWrapper(device, name=device.device_name)
            self.threading = True
        elif device.to_thread == device.TO_THREAD or device.to_thread == True:
            self.threading = True
        self.device = device

        dev_info = dict(on=bool, t=float, step=float)
        self.dev_info = DefDict(dict(drive=dev_info, sense=dev_info, observe_state=dev_info))
        self.dev_info.set(device.config)
        self.dev_step = min(self.dev_info.step())

    def init(self, inpt, state, outpt,):
        self._inpt = inpt
        self._state = state
        self._outpt = outpt

        self.dev_inpt = copy.deepcopy(inpt)
        self.dev_state = copy.deepcopy(state)
        self.dev_outpt = copy.deepcopy(outpt)
        self.dev_timestep = copy.deepcopy(self.timestep)

    def start(self):
        st = perf_counter()
        self.dev_info.t().set([perf_counter() for k in self.dev_info])
        while self.threading:   # if threading is false, then this job simply dies
            if perf_counter() >= st + self.dev_step:
                if self.if_time('drive'):
                    self.device.drive(self.dev_inpt, self.dev_timestep, block=False)
                if self.if_time('sense'):
                    self.dev_outpt.update(self.device.sense(cache=True))
                if self.if_time('observe_state'):
                    self.dev_state.update(self.device.observe_state(cache=True))
                st += self.dev_step
            time.sleep(self.dev_step/10)

    def drive(self,inpt, t, *args, **kwargs):
        if self.dev_info.get('drive').on():
            self.inpt = inpt
            self.timestep = t

    def sense(self, *args, **kwargs):
        if self.dev_info.get('sense').on():
            return self.outpt

    def observe_state(self, *args, **kwargs):
        if self.dev_info.get('observe_state').on():
            return self.state

    def if_time(self, name):
        on = self.dev_info.get(name).on()
        t = self.dev_info.get(name).t()
        step = self.dev_info.get(name).step()
        if on and (perf_counter() >= t + step):
            self.dev_info.get(name).t().set(t+step)
            return True
        return False

    @property
    def inpt(self):
        Lock.acquire()
        self._inpt.set(self.dev_inpt)
        Lock.release()
        return self._inpt

    @inpt.setter
    def inpt(self, val):
        Lock.acquire()
        self.dev_inpt.set(val)
        Lock.release()

    @property
    def state(self):
        Lock.acquire()
        self._state.set(self.dev_state)
        Lock.release()
        return self._state

    @state.setter
    def state(self, val):
        Lock.acquire()
        self.dev_state.set(val)
        Lock.release()

    @property
    def outpt(self):
        Lock.acquire()
        self._outpt.set(self.dev_outpt)
        Lock.release()
        return self._outpt

    @outpt.setter
    def outpt(self, val):
        Lock.acquire()
        self.dev_outpt.set(val)
        Lock.release()

    @property
    def timestep(self):
        Lock.acquire()
        self._timestep = copy.deepcopy(self.dev_timestep)
        Lock.release()
        return self._timestep

    @timestep.setter
    def timestep(self, val):
        Lock.acquire()
        self.dev_timestep = val
        Lock.release()
