import time

from rems.device.DeviceBase import DeviceBase
from rems.sim_handler.ray.RayWrapper import RayWrapper
from time import perf_counter
from rems.typing import DefDict
import copy
from threading import Thread, Lock
from rems.utils import tictoc


class DeviceExecutor(DeviceBase):
    def __init__(self, device, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._dev_inpt = None
        self._dev_state = None
        self._dev_outpt = None
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

    def init(self):
        self.lock = Lock()
        self.device.init()

    def open(self, *args, **kwargs):
        self.device.open()

    def close(self, *args, **kwargs):
        self.threading = False
        self.device.close()

    def start(self):
        next_time = perf_counter()
        self.dev_info.t().set([perf_counter() for k in self.dev_info])
        time.sleep(2)
        while self.threading:   # if threading is false, then this job simply dies
            if perf_counter() >= next_time:
                # st = perf_counter()
                if self.if_time('drive'):
                    self.device.drive(self.dev_inpt, self.dev_timestep, block=False)
                if self.if_time('sense'):
                    self.dev_outpt = self.device.sense(cache=True)
                if self.if_time('observe_state'):
                    s = self.device.observe_state(cache=True)
                    self.dev_state = s
                next_time += self.dev_step
                # print(perf_counter()-st)
            #time.sleep(self.dev_step/10)

    def drive(self, inpt, t, *args, **kwargs):
        if self.dev_info.get('drive').get('on'):
            if self.threading:
                self.dev_inpt = inpt
                self.timestep = t
            else:
                self.device.drive(inpt, t)

    def sense(self, *args, **kwargs):
        if self.dev_info.get('sense').get('on'):
            if self.threading:
                return self.dev_outpt
            else:
                return self.device.sense()

    def observe_state(self, *args, **kwargs):
        if self.dev_info.get('observe_state').get('on'):
            if self.threading:
                return self.dev_state
            else:
                return self.device.observe_state()

    def if_time(self, name):
        on = self.dev_info.get(name).get('on')
        t = self.dev_info.get(name).get('t')
        step = self.dev_info.get(name).get('step')
        if on and (perf_counter() >= t + step):
            self.dev_info.get(name).set({'t': t+step})
            return True
        return False

    @property
    def dev_inpt(self):
        with self.lock:
            ret = self._dev_inpt
            return ret

    @dev_inpt.setter
    def dev_inpt(self, val):
        with self.lock:
            self._dev_inpt = val

    @property
    def dev_state(self):
        with self.lock:
            self._dev_state.set(self.dev_state)
            return self._dev_state

    @dev_state.setter
    def dev_state(self, val):
        with self.lock:
            self._dev_state = val

    @property
    def dev_outpt(self):
        with self.lock:
            return self._dev_outpt

    @dev_outpt.setter
    def dev_outpt(self, val):
        with self.lock:
            self._dev_outpt = val

    @property
    def timestep(self):
        with self.lock:
            self._timestep = self.dev_timestep
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
    from rems.device.iRobot.Create2Device import Create2Device
    from concurrent.futures import ThreadPoolExecutor

    d = DeviceExecutor(Create2Device('COM7'))
    d.init(DefDict(),DefDict(),DefDict())
    d.open()
    d.start()
    d.close()
