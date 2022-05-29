import logging
import asyncio
from sim.job_background.RayJobHandler import JobHandler
import numpy as np
from sim.utils.tictoc import tictoc
from sim.SimActor import SimActor
import ray, time

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
        self.futs = []


    def set_input(self, input_system):
        """set InputSystem
        :param input_system: input to be used (child of InputSystem)"""
        self._input_system = input_system

    def add_robot(self, robot, outputs):
        """Add a robot to simulate and specify output forms
        :param robot: robot to simulate (child of RobotSystem)
        :param outputs: tuple of outputs (child of OutputSystem)
        """
        run = robot.run
        if run.DT is None:
            run.DT = self.DT      # if the robot does not have a specific DT, then provide self.DT
        self.realtime += run.realtime
        r = SimActor.options(name=robot.run.name).remote(robot)
        self._robots.append((robot.inpt, robot, r, outputs))

    def add_process(self, process):
        self._processes.append(process)

    def init_robot(self):
        for inpt, robot, robot_actor, outputs in self._robots:
            init = robot_actor.init.remote()
        ray.get(init)

    def reset_robot(self):
        for inpt, robot, robot_actor, outputs in self._robots:
            robot_actor.reset.remote()
    @tictoc
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
            st = time.perf_counter()
            self.run_robot(t)   # run the robots asynchronously
            self.process()
            t += self.DT
            print(time.perf_counter()-st)
        self.make_outputs()

    def process(self):
        if self._processes:
            for pro in self._processes:
                rets = pro.process()
                self.jHandler.find_job(rets)
            self.jHandler.execute()

    def __del__(self):
        [ray.cancel(f) for f in self.futs]

    @tictoc
    def run_robot(self, t):
        rets= None
        t_start = time.perf_counter()
        if self.futs:
            rets = ray.get(self.futs)
            self.futs.clear()
            print(f"get: {time.perf_counter() - t_start}")

        for inpt, robot, robot_actor, outputs in self._robots:
            inpt.data = self._input_system.get_inputs(inpt, timestamp=t)
            self.futs.append(robot_actor.step_forward.remote(inpt, t, self.DT))
        print(f"Remote call: {time.perf_counter() - t_start}")
        if rets is not None:
            for r, ret in zip(self._robots, rets):
                inpt, robot, robot_actor, outputs = r
                observe, state, info = ret
                robot.outpt.data = observe
                robot.state.data = state
                robot.info = info
                for out in outputs:
                        out.process(state, inpt, observe, t, info)

                if not self.suppress_info:
                    logging.info("Name: {}, dt: {}, t: {}, inpt: {}, state: {}, output: {}, info: {}".format(
                        robot.run.name,
                        np.round(time.perf_counter() - t_start, 5), np.round(t, ROUND),
                        {k: round(v, ROUND) for k, v in inpt.data.items()},
                        {k: round(v, ROUND) for k, v in state.data.items()},
                        {k: round(v, ROUND) for k, v in observe.data.items()},
                    info.data))


    def make_outputs(self):
        for inpt, robot, robot_actor, outputs in self._robots:
            robot.close()
            for out in outputs:
                out.make_output()
