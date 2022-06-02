from sim.job_background.job_type import job_type
from sim.job_background.job_return_type import job_return_type
from sim.job_background.JobHandlerBase import JobHandlerBase

import ray

class RayJobHandler(JobHandlerBase):
    def __init__(self):
        super().__init__()
        self.executor = None

    def execute(self):
        if not self.jobs:
            return
        # submit jobs
        self.futs = [self.task.remote(job) for job in self.jobs]
        self.jobs.clear()

    @staticmethod
    @ray.remote(num_cpus=2)
    def task(job):
        ret = job.job()
        if isinstance(ret, job_return_type):
            ret.callback(**ret.kwargs)

    def __del__(self):
        [ray.cancel(f) for f in self.futs]

