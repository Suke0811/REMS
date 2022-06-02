import copy
import logging
import queue

from sim.job_background import JobHandler as JobHandler
import numpy as np
from sim.utils.tictoc import tictoc
from sim.SimActor import SimActor
import ray, time, signal
from ray.util.queue import Queue, Empty
from sim.ray import ProcessActor

ROUND = 2

class Sim:
    DT_ERR = 0.01
    DT_DEFAULT = 0.25

    def __init__(self, suppress_info=False, DT=DT_DEFAULT):
        """
        :param suppress_info: no log outputs to console
        """
        self.suppress_info = suppress_info
        self._input_system = None
        self._robots = []
        self._robots_input = []
        self._processes = []
        self.jHandler = JobHandler()
        self.realtime = False
        self.DT = DT
        self.futs = []
        self.queues =[]
        self.q_ins = []
        self.q_outs = []
        self.robot_actors = []
        self.inpts = []
        self.futs_time = []
        self.futs_robots = []
        self.t_process = []
        signal.signal(signal.SIGINT, self.handler_ctrl_c)
        self._processes_refs= []
        #ray.init(local_mode=True)

    def handler_ctrl_c(self, signum, frame):
        self.close()
        exit(1)

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
        q_in = Queue()
        q_out = Queue()
        self.queues.append((q_in, q_out))
        r = SimActor.options(name=run.name+str(time.time()), max_concurrency=2).remote(robot, q_in, q_out)
        self._robots.append((robot.inpt, robot, r, outputs))
        self.q_ins.append(q_in)
        self.q_outs.append(q_out)
        self.robot_actors.append(r)
        self.inpts.append(robot.inpt)


    def add_process(self, process):
        self._processes.append(ProcessActor.options(max_concurrency=2).remote(process))


    def init(self):
        futs = []
        for inpt, robot, robot_actor, outputs in self._robots:
            futs.append(robot_actor.init.remote())
        done = ray.get(futs)

    def reset(self, t):
        futs = []
        for inpt, robot, robot_actor, outputs in self._robots:
            inpt.data = self._input_system.get_inputs(inpt, timestamp=t)
            futs.append(robot_actor.reset.remote(inpt, t))
        done = ray.get(futs)
        time.sleep(1)

    def close(self):
        futs = []
        for inpt, robot, robot_actor, outputs in self._robots:
            futs.append(robot_actor.close.remote())
        done = ray.get(futs)
        time.sleep(0.5)

    @tictoc
    def run(self, max_duration, realtime=True):
        """Run robots with the given settings for max_duration seconds
        :param max_duration: the maximum duration of test
        """
        self.realtime += realtime
        t = 0
        if self._input_system is None:
            raise ImportError('Input is required')   # you need to have one InputSystem

        self.init()
        self.reset(t)
        #[robot_actor.main.remote() for inpt, robot, robot_actor, outputs in self._robots]
        st = time.perf_counter()
        next_time = time.perf_counter()
        while t <= max_duration and not self._input_system.quite:
           # self.step(t)   # run the robots asynchronously
           if self.realtime is False or time.perf_counter() >= next_time:
                self.run_robot(t)
                self.process()
                t += self.DT
                next_time += self.DT

        print(f"loop time {time.perf_counter()-st}")
        self.make_outputs()
        self.close()
        print(f"close {time.perf_counter() - st}")

    def step(self, t):
        #for inpt, r, q_in in zip(self.inpts, self.robot_actors, self.q_ins):
        inpt = self.inpts[0]
        inpt.data = self._input_system.get_inputs(inpt, timestamp=t)
        for i in range(2):
            q_in = self.q_ins[i]
            q_in.put((inpt, t, self.DT))
        rets = [q_out.get() for q_in, q_out in self.queues]

        if rets is not None:
            for r, ret in zip(self._robots, rets):
                inpt, robot, robot_actor, outputs = r
                observe, state, info, dt_actual = ret
                robot.outpt.data = observe
                robot.state.data = state
                robot.info.data = info
                for out in outputs:
                    out.process(robot.state, inpt, robot.outpt, t, robot.info)

                if not self.suppress_info:
                    logging.info("Name: {}, dt: {}, t: {}, inpt: {}, state: {}, output: {}, info: {}".format(
                        robot.run.name,
                        np.round(dt_actual, 5), np.round(t, ROUND),
                        {k: round(v, ROUND) for k, v in inpt.data.items()},
                        {k: round(v, ROUND) for k, v in robot.state.data.items()},
                        {k: round(v, ROUND) for k, v in robot.outpt.data.items()},
                        robot.info.data))

    @tictoc
    def process(self):
        return
        if self._processes:
            if self._processes_refs:
                finished, self._processes_refs = ray.wait(self._processes_refs, num_returns=len(self._processes_refs))
                if finished:
                    rets = (ray.get(finished[-1]))

            for pro in self._processes:
                self._processes_refs.append(pro.process.remote())


    def run_robot(self, t):
        self.get_ret()
        # self.inpts[0].data = self._input_system.get_inputs(self.inpts[0], timestamp=t)
        # self.futs.append(self.robot_actors[0].step_forward.remote(self.inpts[0], t, self.DT))
        for inpt, robot, robot_actor, outputs in self._robots:
            inpt.data = self._input_system.get_inputs(inpt, timestamp=t)
            self.futs.append(robot_actor.step_forward.remote(inpt, t, self.DT))
            self.futs_time.append(t)
            self.futs_robots.append((inpt, robot, robot_actor, outputs))

    def get_ret(self):
        if self.futs:
            finished, self.futs = ray.wait(self.futs, num_returns=len(self.robot_actors))
            futs_time = self.futs_time[0:len(finished)]
            fut_robot = self.futs_robots[0:len(finished)]
            self.futs_time = self.futs_time[len(finished):-1]
            self.fut_robot = self.futs_robots[len(finished):-1]
            for t, f, f_r in zip(futs_time, finished, fut_robot):
                self.return_handle(ray.get(f), t, f_r)

    def return_handle(self, ret, t, ret_robot):
        inpt, robot, robot_actor, outputs = ret_robot
        observe, state, info, dt_actual = ret
        robot.outpt.data = observe
        robot.state.data = state
        robot.info.data = info
        for out in outputs:
            out.process(robot.state, inpt, robot.outpt, t, robot.info)

        if not self.suppress_info:
            logging.info("Name: {}, dt: {}, t: {}, inpt: {}, state: {}, output: {}, info: {}".format(
                robot.run.name,
                np.round(dt_actual, 5), np.round(t, ROUND),
                {k: round(v, ROUND) for k, v in inpt.data.items()},
                {k: round(v, ROUND) for k, v in robot.state.data.items()},
                {k: round(v, ROUND) for k, v in robot.outpt.data.items()},
            robot.info.data))


    def make_outputs(self):
        for inpt, robot, robot_actor, outputs in self._robots:
            for out in outputs:
                out.make_output()
