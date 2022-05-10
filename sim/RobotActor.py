import ray
import time, threading
import asyncio

@ray.remote(num_cpus=2)
class RobotActor:
    def __init__(self, robot, out, frequency=100):
        self.robot = robot
        self.out = out
        self.pause = asyncio.Event()
        self.frequency = frequency

    def init(self):
        self.robot.init()

    def reset(self):
        self.robot.reset()

    def step_forward(self, inpt, t_init, dt):
        t_start = time.time()
        t=t_init
        state = None
        observe = None
        info = None
        while t - t_init < dt:
            state = self.robot.drive(inpt, t)
            observe = self.robot.sense()

            t = self.robot.clock(t)
            info = self.robot.get_update()
        for out in self.out:
                out.process(state, inpt, observe, t, info)

    def run_resume(self, clear=False):
        self.pause.set()
        if clear:
            self.pause.clear()

    async def run_pause(self):
        await self.ready_event.wait()


