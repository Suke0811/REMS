import ray
from sim.sim_handler.job_background import JobHandler


@ray.remote(num_cpus=1)
class ProcessActor:
    def __init__(self, process, *args, **kwargs):
        self.process = process(*args, **kwargs)
        self.jHandler = JobHandler()

    def process(self):
        rets = self.process.process()
        self.jHandler.find_job(rets)
        self.jHandler.execute()

