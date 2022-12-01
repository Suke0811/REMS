import ray
from rems.sim_handler.job_background import JobHandler as JobHandler
from rems.sim_handler.ray.RobotRayWrapper import RobotRayWrapper


@ray.remote
class ProcessActor:
    def __init__(self, process, *args, **kwargs):
        for a in args:
            if isinstance(a, RobotRayWrapper):
                a._reset_attr()
        self.process_system = process(*args)
        self.jHandler = JobHandler()

    def init(self, *args, **kwargs):
        pass

    def process(self, t):
        rets = self.process_system.process(t)
        self.jHandler.find_job(rets)
        self.jHandler.execute()

