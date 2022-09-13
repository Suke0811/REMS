import logging

from sim.sim_handler.job_background import JobHandler as JobHandler
import numpy as np
from sim.utils import tictoc, time_str
from sim.sim_handler.ray.SimActor import SimActor
import ray, time, signal
from sim.sim_handler.ray import ProcessActor
from sim.outputs import FileCsvOutput
from sim.Config import SimConfig
from sim.robots.bind_robot import bind_robot
from sim.sim_handler.ray import RobotRayWrapper as ray_robot
ROUND = 2

class Simulation:
    DT_ERR = 0.01
    DT_DEFAULT = 0.25

    def __init__(self, debug_mode=False, suppress_info=False, ray_init_options: dict=None):
        """
        :param suppress_info: no log outputs to console
        """
        if ray_init_options is not None:
            ray_init_options.setdefault('local_mode', debug_mode)
            ray_init_options.setdefault('num_gpus', 1)
            ray.init(**ray_init_options)
        else:
            ray.init(local_mode=debug_mode, num_gpus=1) # for windows, not setting num_gpu cause an error


        self.suppress_info = suppress_info
        self._input_system = None
        self._robots = []
        self._robots_input = []
        self._processes = []
        self.jHandler = JobHandler()
        self.realtime = False
        self.runconfig = None
        self.DT = None
        self.futs = []
        self.robot_actors = []
        self.futs_time = []
        self.futs_robots = []
        signal.signal(signal.SIGINT, self.handler_ctrl_c)
        self._processes_refs= []
        self.made_outputs = False

    def handler_ctrl_c(self, signum, frame):
        """Ctrl+C termination handling"""
        self.close()
        self.make_outputs()
        exit(1)

    def set_input(self, input_system):
        """set InputSystem
        :param input_system: input to be used (child of InputSystem)"""
        input_system.init()
        self._input_system = input_system

    def add_robot(self, robot_def, robot, outputs=None, inpt=None):
        """
        Add a robot to simulate and specify output forms
        :param robot: robot to simulate (child of RobotSystem)
        :param outputs: tuple of outputs (child of OutputSystem)
        :param inpt: InputSystem specific to the robot. Default to system wide Inputsystem
        """

        robot = bind_robot(robot_def, robot)
        run = robot.run
        self.realtime += run.realtime
        if outputs is None:     # in no output is specified, then do file outputs
            outputs = FileCsvOutput('out/' + robot.run.name + '_' + time_str() + '.csv')
        if not isinstance(outputs, tuple):
            outputs = (outputs,)

        r = ray_robot(robot, outputs)

        self._robots.append((inpt, robot, r, outputs))
        self.robot_actors.append(r)
        if self._input_system is None and inpt is not None:
            self._input_system = inpt
        return r  # robot reference (virtually the same as the robot)

    def add_process(self, process, *args):
        r, r2, t = args
        print(r.state)
        self._processes.append(ProcessActor.options(max_concurrency=2).remote(process, *args))

    def init(self):
        futs = []
        for inpt, robot, robot_actor, outputs in self._robots:
            futs.append(robot_actor.init_devices(block=False))
        done = ray.get(futs)

        for inpt, robot, robot_actor, outputs in self._robots:
            futs.append(robot_actor.init(block=False))
        done = ray.get(futs)

    def open(self):
        futs = []
        for inpt, robot, robot_actor, outputs in self._robots:
            futs.append(robot_actor.open(block=False))
        done = ray.get(futs)

    def reset(self, t):
        futs = []
        for inpt, robot, robot_actor, outputs in self._robots:
            if inpt is None:
                robot.state.set(self._input_system.get_inputs(robot.state, timestamp=t, prefix='state'))
            else:
                robot.state.set(inpt.get_inputs(robot.state, timestamp=t, prefix='state'))
            futs.append(robot_actor.reset(robot.state, t, block=False))
        done = ray.get(futs)
        time.sleep(1)

    def close(self):
        futs = []
        for inpt, robot, robot_actor, outputs in self._robots:
            futs.append(robot_actor.close(block=False))
        done = ray.get(futs)
        time.sleep(0.5)

    def set_dt(self, DT):
        self.DT = DT
        futs = []
        for r in self.robot_actors:
            futs.append(r.set_DT(self.DT, block=False))
        done = ray.get(futs)



    @tictoc
    def run(self, config: SimConfig):
        """Run robots with the given settings for max_duration seconds
        :param max_duration: the maximum duration of test
        """
        self.runconfig = config
        self.set_dt(config.dt)
        self.realtime += config.realtime

        t = config.start_time
        if self._input_system is None:
            raise ImportError('Input is required')   # you need to have one InputSystem
        self.init()
        self.open()
        self.reset(t)
        st = time.perf_counter()
        next_time = time.perf_counter()
        while not config.if_time(t) and not self._input_system.quite:
            if self.realtime == False or time.perf_counter() >= next_time:
                self.step(t)
                self.process(t)
                t += self.DT
                next_time += self.DT / config.run_speed     # manipulate run speed
        logging.info(f"loop time {time.perf_counter()-st}")
        self.close()
        self.make_outputs()
        # self.close()

    def step(self, t):
        for inpt, robot, robot_actor, outputs in self._robots:
            if inpt is None:
                i = self._input_system.get_inputs(robot.inpt, timestamp=t)
            else:
                i = inpt.get_inputs(robot.inpt, timestamp=t)
            robot_actor.step(i, t, self.DT, block=False)

    def process(self, t):
        if self._processes:

            #if self._processes_refs:
            # this wait takes 0.04sec?
                #finished, self._processes_refs = ray.wait(self._processes_refs, num_returns=len(self._processes_refs))
                #if finished:
                    #pass
                    #rets = (ray.get(finished[-1]))
            for pro in self._processes:
                self._processes_refs.append(pro.process.remote(t))


    def run_robot(self, t):
        self.get_ret()
        for inpt, robot, robot_actor, outputs in self._robots:
            if inpt is None:
                i = self._input_system.get_inputs(robot.inpt, timestamp=t)
            else:
                i = inpt.get_inputs(robot.inpt, timestamp=t)
            #######
            self.futs.append(robot_actor.step_forward.remote(i, t, self.DT))
            self.futs_time.append(t)
            self.futs_robots.append((robot.inpt, robot, robot_actor, outputs))

            ####### ~0.0001s

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
        robot.outpt.set(observe)
        robot.state.set(state)
        robot.info.set(info)
        for out in outputs:
            out.process(robot.state, robot.inpt, robot.outpt, t, robot.info)

        if not self.suppress_info:
            logging.info("Name: {}, dt: {}, t: {}, inpt: {}, state: {}, output: {}, info: {}".format(
                robot.run.name,
                np.round(dt_actual, 5), np.round(t, ROUND),
                {k: round(v, ROUND) for k, v in inpt.items()},
                {k: round(v, ROUND) for k, v in robot.state.items()},
                {k: round(v, ROUND) for k, v in robot.outpt.items()},
            robot.info))

    def make_outputs(self):
        # right now make outputs can be called only once
        if not self.made_outputs:
            futs = []
            for inpt, robot, robot_actor, outputs in self._robots:
                futs.append(robot_actor.make_outputs(block=False))
            done = ray.get(futs)
            self.made_outputs = True


