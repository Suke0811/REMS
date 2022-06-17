import ray
from sim.sim_handler.job_background import JobHandler
from sim.sim_handler.ray.RobotRayWrapper import RobotRayWrapper


@ray.remote
class ProcessActor:
    def __init__(self, process, *args):
        for a in args:
            if isinstance(a, RobotRayWrapper):
                a._reset_attr()
        self.process = process(*args)
        self.jHandler = JobHandler()

    def process(self, t):
        rets = self.process.process(t)
        self.jHandler.find_job(rets)
        self.jHandler.execute()

