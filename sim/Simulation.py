import time
import logging
import ray

from sim.job_background.RayJobHandler import RayJobHandler as JobHandler
from sim.robot_actor.RobotActor import RobotActor
from sim.robot_actor.definition_queue import *
from ray.util.queue import Queue, Empty
from sim.type import DefDict

from sim.Sim import Sim as SimBase
import numpy as np

ROUND = 2

class Sim(SimBase):
    DT_ERR = 0.01
    DT_DEFAULT = 0.25

    def __init__(self, suppress_info=False, DT=DT_DEFAULT):
        """
        :param suppress_info: no log outputs to console
        """
        self.suppress_info = suppress_info
        self._input_system = None
        self._robots = []
        self._outs = []
        self._robot_actors = []
        self._queue_ins = []
        self._data_in = DefDict(QUEUE_IN)
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
        self._robots.append(robot)
        self._outs.append(outputs)
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
            self._robot_actors.append(RobotActor.remote(robot, outputs, q_in, q_out))

    def init(self):
        self.unpack_robot()
        for r in self._robot_actors:
            r.init.remote()

    def reset(self):
        for r in self._robot_actors:
            r.reset.remote()

    def start_robot(self):
        [r.main_loop.remote() for r in self._robot_actors]

    def run(self, max_duration, realtime=True):
        """Run modules with the given settings for max_duration seconds
        :param max_duration: the maximum duration of test
        """
        self.realtime += realtime

        if self._input_system is None:
            raise ImportError('Input is required')   # you need to have one InputSystem
        self.init()
        self.reset()
        self.start_robot()
        t = 0
        while t < max_duration and not self._input_system.is_done():
            # run the modules asynchronously
            self.send_data(t)
            self.receive_data() # blocks till we get data
            self.process()
            t += self.DT
        self.make_outputs()

    def process(self):
        if self._processes:
            for pro in self._processes:
                rets = pro.process()
                self.jHandler.find_job(rets)
            self.jHandler.execute()

    def send_data(self, t):  # sending data won't block the process
        self._data_in.data_as([self._input_system.get_inputs(timestamp=t), t, self.DT])
        for q, r in zip(self._queue_ins, self._robots):
            q.put_nowait(self._data_in.data_as(r))

    def receive_data(self):
        # only receive data when all of the queue is not empty
        data = [q.get() for q in self._queue_outs]
        for d, r, o in zip(data, self._robots, self._outs):
            """Receiving data can block the process if necessary"""
            robot = d.data.get(ROBOT)
            out = d.data.get(OUTPT)
            r.overwrite_robot(robot.state, robot.outpt)     #TODO: maybe change the way to update the local instance of the robot?
            o = out

    def make_outputs(self):
        for robot, outputs in self._robots:
            for out in outputs:
                out.make_output()
