import logging

from rems.sim_handler.job_background import JobHandler as JobHandler
import numpy as np
import ray, time, signal
from rems.sim_handler.ray import ProcessActor
from rems.Config import SimConfig
from rems.robots.bind_robot import bind_robot
from rems.sim_handler.ray import RobotRayWrapper as ray_robot
ROUND = 2

class Operator:
    DT_ERR = 0.01
    DT_DEFAULT = 0.25

    def __init__(self, debug_mode=False, suppress_info=False, ray_init_options: dict=None, runtime_env: dict = None):
        """
        :param suppress_info: no log outputs to console
        """
        if ray_init_options is None:
            ray_init_options = {}
        ray_init_options.setdefault('local_mode', debug_mode)
        ray_init_options.setdefault('num_gpus', 1)  # for windows, not setting num_gpu cause an error
        if runtime_env is not None:
            ray_init_options.setdefault('runtime_env', runtime_env)  # for remote cluster environment
        ray.init(**ray_init_options)

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

    def add_robot(self, robot_def=None, robot=None, def_args=None, robot_args=None, outputs=None, inpt=None, remote_ip=None):
        """
        Add a robot to simulate and specify output forms
        :param robot: robot to simulate (child of RobotSystem)
        :param outputs: tuple of outputs (child of OutputSystem)
        :param inpt: InputSystem specific to the robot. Default to system wide Inputsystem
        """
        robot = bind_robot(robot_def, robot, def_args, robot_args)
        run = robot.run
        self.realtime += run.realtime
        # if outputs is None:     # in no output is specified, then do file outputs
        #     outputs = FileCsvOutput('out/' + robot.run.name + '_' + time_str() + '.csv')
        if not isinstance(outputs, tuple):
            outputs = (outputs,)

        r = ray_robot(robot, outputs, remote_ip=remote_ip)

        self._robots.append((inpt, robot, r, outputs))
        self.robot_actors.append(r)
        if self._input_system is None and inpt is not None:
            self._input_system = inpt
        return r  # robot reference (virtually the same as the robot)

    def add_process(self, process, *args, **kwargs):
        p = ProcessActor.options(max_concurrency=2).remote(process, *args, **kwargs)
        self._processes.append(p)
        return p

    def init(self, t):
        futs = []

        for inpt, robot, robot_actor, outputs in self._robots:
            futs.append(robot_actor.init_devices(block=False))
        done = ray.get(futs)

        for inpt, robot, robot_actor, outputs in self._robots:
            state = None
            if inpt is None:
                ret = self._input_system.get_inputs(timestamp=t, prefix='state')
            else:
                ret = inpt.get_inputs(timestamp=t, prefix='state')
                if ret is not None:
                    state = robot.state.set(ret)
            if state is None:
                state = robot_actor.state
            futs.append(robot_actor.init(init_state=state, block=False))
        done = ray.get(futs)


        time.sleep(1)
    def init_process(self):
        for p in self._processes:
            p.init.remote()

    def open(self):
        futs = []
        for inpt, robot, robot_actor, outputs in self._robots:
            futs.append(robot_actor.open(block=False))
        done = ray.get(futs)

    def reset(self, t):
        futs = []
        for inpt, robot, robot_actor, outputs in self._robots:
            state = None
            if inpt is None:
                ret = self._input_system.get_inputs(timestamp=t, prefix='state')
                if ret is not None:
                    state = robot.state.set(ret)
            else:
                ret = inpt.get_inputs(timestamp=t, prefix='state')
                if ret is not None:
                    state = robot.state.set(ret)
            futs.append(robot_actor.reset(state, t, block=False))
        done = ray.get(futs)
        time.sleep(1)

    def close(self):
        futs = []
        for inpt, robot, robot_actor, outputs in self._robots:
            futs.append(robot_actor.drive(robot.inpt.to_default(), 0, block=False))
            futs.append(robot_actor.close(block=False))
        done = ray.get(futs)
        time.sleep(0.5)

    def set_dt(self, DT):
        self.DT = DT
        futs = []
        for r in self.robot_actors:
            futs.append(r.set_DT(self.DT, block=False))
        done = ray.get(futs)

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
        self.init(t)
        self.init_process()
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

    def step(self, t):
        for inpt, robot, robot_actor, outputs in self._robots:
            if inpt is None:
                i = self._input_system.get_inputs(timestamp=t)
            else:
                i = inpt.get_inputs(timestamp=t)
            robot_actor.step(i, t, self.DT, block=False)

    def process(self, t):
        if self._processes:
            for pro in self._processes:
                self._processes_refs.append(pro.process.remote(t))

    def return_handle(self, ret, t, ret_robot):
        inpt, robot, robot_actor, outputs = ret_robot
        observe, state, info, dt_actual = ret
        robot.outpt.set(observe)
        robot.state.set(state)
        robot.info.set(info)
        for out in outputs:
            out.process(robot.state, robot.inpt, robot.outpt, t, robot.info)

        if not self.suppress_info:
            logging.info("Name: {}, t: {}, inpt: {}, state: {}, output: {}, info: {}, dt: {},".format(
                robot.run.name,
                {k: round(v, ROUND) for k, v in inpt.items()},
                {k: round(v, ROUND) for k, v in robot.state.items()},
                {k: round(v, ROUND) for k, v in robot.outpt.items()},
            robot.info,
            np.round(dt_actual, 5), np.round(t, ROUND),))

    def make_outputs(self):
        # right now make outputs can be called only once
        if not self.made_outputs:
            futs = []
            for inpt, robot, robot_actor, outputs in self._robots:
                futs.append(robot_actor.make_outputs(block=False))
            done = ray.get(futs)
            self.made_outputs = True


