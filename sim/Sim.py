from time import perf_counter as time
import logging
import asyncio
from sim.job_background.RayJobHandler import JobHandler
import numpy as np
import math

ROUND = 2

class Sim:
    DT_ERR = 0.01
    DT_DEFAULT = 0.25

    def __init__(self, use_noise=False, suppress_info=False, DT=DT_DEFAULT):
        """
        :param suppress_info: no log outputs to console
        """
        self._use_noise = use_noise
        self.suppress_info = suppress_info
        self._input_system = None
        self._robots = []
        self._robots_input = []
        self._processes = []
        self.jHandler = JobHandler()
        self.realtime = False
        self.DT = DT


    def set_input(self, input_system):
        """set InputSystem
        :param input_system: input to be used (child of InputSystem)"""
        self._input_system = input_system

    def add_robot(self, robot, outputs):
        """Add a robot to simulate and specify output forms
        :param robot: robot to simulate (child of RobotSystem)
        :param outputs: tuple of outputs (child of OutputSystem)
        """
        self._robots.append([robot.inpt, robot, outputs])
        run = robot.run
        if run.DT is None:
            run.DT = self.DT      # if the robot does not have a specific DT, then provide self.DT
        self.realtime += run.realtime

    def add_process(self, process):
        self._processes.append(process)

    def init_robot(self):
        for inpt, robot, outputs in self._robots:
            robot.init()

    def reset_robot(self):
        for inpt, robot, outputs in self._robots:
            robot.reset()

    def run(self, max_duration, realtime=True):
        """Run robots with the given settings for max_duration seconds
        :param max_duration: the maximum duration of test
        """
        self.realtime += realtime
        t = 0
        if self._input_system is None:
            raise ImportError('Input is required')   # you need to have one InputSystem

        self.init_robot()
        self.reset_robot()

        while t < max_duration and not self._input_system.quite:
            asyncio.run(self.create_tasks(t))   # run the robots asynchronously
            self.process()
            t += self.DT
        self.make_outputs()

    def process(self):
        if self._processes:
            for pro in self._processes:
                rets = pro.process()
                self.jHandler.find_job(rets)
            self.jHandler.execute()


    async def create_tasks(self, t):
        sim_tasks = []  # list of tasks (threads)
        for inpt, robot, outputs in self._robots:
            inpt.data = self._input_system.get_inputs(inpt, timestamp=t)
            if robot.run.to_thread: # send the robot process to a separate thread
                sim_tasks.append(asyncio.to_thread(self.step_forward, inpt, robot, outputs, t))
            else:
                sim_tasks.append(asyncio.wait_for(self.step_forward(inpt, robot, outputs, t), self.DT-self.DT_ERR))

        if self.realtime:
            sim_tasks.append(asyncio.sleep(self.DT-self.DT_ERR))
        try:
            await asyncio.wait_for((asyncio.gather(*sim_tasks, return_exceptions=True)), self.DT)  # awaits all threads are done
        except asyncio.TimeoutError:
            logging.info('timeout happened. DT of the actual simulation is bigger than specified DT') # forcing to time out for every DT. Still we get the return

    def step_forward(self, inpt, robot, outputs, t_init):
        t_start = time()
        t=t_init
        state = None
        observe = None
        info = None
        while t - t_init < self.DT:
            robot.drive(inpt, t)
            observe = robot.sense()
            state = robot.observe_state()

            t = robot.clock(t)
            info = robot.info

        for out in outputs:
                out.process(state, inpt, observe, t, info)

        if not self.suppress_info:
            logging.info("dt: {}, t: {}, inpt: {}, state: {}, output: {}, info: {}".format(
                np.round(time() - t_start, 5), np.round(t, ROUND),
                {k: round(v, ROUND) for k, v in inpt.data.items()},
                {k: round(v, ROUND) for k, v in state.data.items()},
                {k: round(v, ROUND) for k, v in observe.data.items()},
            info.data))

    def make_outputs(self):
        for inpt, robot, outputs in self._robots:
            robot.close()
            for out in outputs:
                out.make_output()
