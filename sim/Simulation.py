import time
import logging
import asyncio, threading

import ray

from sim.job_background.RayJobHandler import JobHandler
from sim import RobotActor
from ray.util.queue import Queue, Empty
import numpy as np

DT = 0.1

class Sim:
    DT_ERR = 0.01

    def __init__(self, suppress_info=False):
        """
        :param suppress_info: no log outputs to console
        """
        self.suppress_info = suppress_info
        self._input_system = None
        self._robots = []
        self._robot_actors = []
        self._queue_ins = []
        self._queue_outs = []
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
        self._robots.append([robot, outputs])
        run = robot.run
        if run.DT is None:
            run.DT = self.DT      # if the robot does not have a specific DT, then provide TEST.DT
        self.realtime += run.realtime

    def add_process(self, process):
        self._processes.append(process)

    def unpack_robot(self):
        for robot, outputs in self._robots:
            q_in = Queue()
            q_out = Queue()
            self._queue_ins.append(q_in)
            self._queue_outs.append(q_out)
            self._robot_actors.append(RobotActor(robot, outputs, q_in, q_out))

    def init_robot(self):
        for r in self._robot_actors:
            r.init.remote()

    def reset_robot(self):
        for r in self._robot_actors:
            r.init.remote()

    def run(self, max_duration, realtime=True):
        """Run modules with the given settings for max_duration seconds
        :param max_duration: the maximum duration of test
        """
        self.realtime += realtime

        if self._input_system is None:
            raise ImportError('Input is required')   # you need to have one InputSystem
        self.init_robot()
        self.reset_robot()
        t = 0
        while t < max_duration and not self._input_system.is_done():
            # run the modules asynchronously
            asyncio.run(self.create_tasks(t))  # run the robots asynchronously
            t += self.DT
        self.make_outputs()

    def process(self):
        if self._processes:
            for pro in self._processes:
                rets = pro.process()
                self.jHandler.find_job(rets)
            self.jHandler.execute()
            self.jHandler.execute()

    def make_outputs(self):
        for robot, outputs in self._robots:
            for out in outputs:
                out.make_output()

    async def create_tasks(self, t):
        self.inpt = self._input_system.get_inputs(timestamp=t)
        sim_tasks = []  # list of tasks (threads)
        sim_tasks.append(asyncio.wait_for(self.loop(t), TEST.DT-self.DT_ERR))
        if self.realtime:
            sim_tasks.append(asyncio.sleep(TEST.DT-self.DT_ERR))
        try:
            await asyncio.wait_for((asyncio.gather(*sim_tasks, return_exceptions=True)), TEST.DT)  # awaits all threads are done
        except asyncio.TimeoutError:
            logging.info('timeout happened. DT of the actual simulation is bigger than specified DT') # forcing to time out for every DT. Still we get the return

    def loop(self, t):
        for robot_actor in self._robot_actors:
            robot_actor.step_forward.remote(self.inpt, t, self.DT)
        self.process()
