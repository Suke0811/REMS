import threading
import time

from sim.device.DeviceBase import DeviceBase
from sim.sim_handler.ray.RayWrapper import RayWrapper
from time import perf_counter
from sim.typing import DefDict
import copy
from sim.utils import tictoc
from threading import Thread, Lock


class DeviceExecutor(DeviceBase):
    def __init__(self, device, *args, **kwargs):
        self._inpt = None
        self._state = None
        self._outpt = None
        self._timestep = 0.0


        if device.to_thread == device.TO_PROCESS:
            device = RayWrapper(device, name=device.device_name)
            self.threading = True
        elif device.to_thread == device.TO_THREAD or device.to_thread == True:
            self.threading = True
        else:
            self.threading = False
        self.device = device

        dev_info = dict(on=bool, t=float, step=float)
        self.dev_info = DefDict(dict(drive=dev_info, sense=dev_info, observe_state=dev_info))
        self.dev_info.set(device.config)
        self.dev_step = min(self.dev_info.step())
        self.dev_timestep = 0.0

    def init(self, inpt, state, outpt,):
        self.lock = Lock()
        self._inpt = inpt
        self._state = state
        self._outpt = outpt

        self.dev_inpt = copy.deepcopy(inpt)
        self.dev_state = copy.deepcopy(state)
        self.dev_outpt = copy.deepcopy(outpt)

        self.device.init()

    def open(self, *args, **kwargs):
        self.device.open()


    def close(self, *args, **kwargs):
        self.threading = False
        self.device.close()

    def start(self):
        next_time = perf_counter()
        self.dev_info.t().set([perf_counter() for k in self.dev_info])
        while self.threading:   # if threading is false, then this job simply dies
            if perf_counter() >= next_time:
                # st = perf_counter()
                if self.if_time('drive'):
                    self.device.drive(self.dev_inpt, self.dev_timestep, block=False)
                if self.if_time('sense'):
                    self.dev_outpt.update(self.device.sense(cache=True))
                if self.if_time('observe_state'):
                    s = self.device.observe_state(cache=True)
                    self.dev_state.update(s)
                next_time += self.dev_step
                # print(perf_counter()-st)
            time.sleep(self.dev_step/10)

    def drive(self, inpt, t, *args, **kwargs):
        if self.dev_info.get('drive').get('on'):
            if self.threading:
                self.dev_inpt.update(inpt)
                self.timestep = t
            else:
                self.device.drive(inpt, t)

    def sense(self, *args, **kwargs):
        if self.dev_info.get('sense').get('on'):
            if self.threading:
                return self.outpt
            else:
                return self.outpt.update(self.device.sense())

    def observe_state(self, *args, **kwargs):
        if self.dev_info.get('observe_state').get('on'):
            if self.threading:
                return self.state
            else:
                return self.state.update(self.device.observe_state())

    def if_time(self, name):
        on = self.dev_info.get(name).get('on')
        t = self.dev_info.get(name).get('t')
        step = self.dev_info.get(name).get('step')
        if on and (perf_counter() >= t + step):
            self.dev_info.get(name).set({'t': t+step})
            return True
        return False

    @property
    def inpt(self):
        with self.lock:
            self._inpt.set(self.dev_inpt)
            return self._inpt

    @inpt.setter
    def inpt(self, val):
        with self.lock:
            self.dev_inpt.set(val)

    @property
    def state(self):
        with self.lock:
            self._state.set(self.dev_state)
            return self._state

    @state.setter
    def state(self, val):
        with self.lock:
            self.dev_state.set(val)

    @property
    def outpt(self):
        with self.lock:
            self._outpt.set(self.dev_outpt)
            return self._outpt

    @outpt.setter
    def outpt(self, val):
        with self.lock:
            self.dev_outpt.set(val)

    @property
    def timestep(self):
        with self.lock:
            self._timestep = copy.deepcopy(self.dev_timestep)
            return self._timestep

    @timestep.setter
    def timestep(self, val):
        with self.lock:
            self.dev_timestep = val


class DeviceHock(DeviceBase):
    device_name = 'Hock'
    def __init__(self, hock, drive=False, sense=False, observe_state=False):
        self.hock = hock
        self.config.on().set([drive, sense, observe_state])
        self.to_thread = self.TO_THREAD

    def drive(self, inpt: DefDict, timestamp, *args, **kwargs):
        pass

    def sense(self, *args, **kwargs):
        pass

    def observe_state(self, *args, **kwargs):
        pass






if __name__ == '__main__':
    from sim.device.iRobot.Create2Device import Create2Device
    from concurrent.futures import ThreadPoolExecutor

    d = DeviceExecutor(Create2Device('COM7'))
    d.init(DefDict(),DefDict(),DefDict())
    d.open()
    d.start()
    d.close()
