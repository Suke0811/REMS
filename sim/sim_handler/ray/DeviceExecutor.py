from sim.device.DeviceBase import DeviceBase
from sim.sim_handler.ray.RayWrapper import RayWrapper
from time import perf_counter
from sim.typing import DefDict




class DeviceExecutor:
    def __init__(self, device: DeviceBase, dev_step, timestep, inpt, state, output):
        self.config = device.config
        self.threading = False
        if device.to_thread == device.TO_PROCESS:
            device = RayWrapper(device, name=device.device_name)
        elif device.to_thread == device.TO_THREAD or device.to_thread == True:
            self.threading = True
        self.device = device

        self.inpt = inpt
        self.state = state
        self.output = output
        self.timestep = timestep
        self.dev_step = dev_step
        dev_info = dict(on=bool, t=float, step=float)
        self.dev_info = DefDict(dict(drive=dev_info, sense=dev_info, observe_state=dev_info))
        self.quit = False

    def start(self):
        st = perf_counter()
        self.dev_info.t().set([perf_counter() for k in self.dev_info])
        while not self.quit:
            if perf_counter() >= st + self.dev_step:
                if self.if_time('drive'):
                    self.device.drive(self.inpt, self.timestep)
                if self.if_time('sense'):
                    self.output.update(self.device.sense())
                if self.if_time('observe_state'):
                    self.state.update(self.device.observe_state())
                st += self.dev_step

    def drive(self,inpt, t):
        self.inpt = inpt
        self.timestep = t

    def sense(self,*args, **kwargs):
        return self.output

    def observe_state(self,*args, **kwargs):
        return self.state


    def if_time(self, name):
        on = self.dev_info.get(name).on()
        t = self.dev_info.get(name).t()
        step = self.dev_info.get(name).step()
        if on and (perf_counter() >= t + step):
            self.dev_info.get(name).t().set(t+step)
            return True
        return False
