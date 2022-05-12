from sim.job_background.job_type import job_type
from sim.job_background.job_return_type import job_return_type
from sim.job_background.JobHandler import JobHandler
from concurrent.futures import ThreadPoolExecutor

import ray

class RayJobHandler(JobHandler):
    def __init__(self):
        super().__init__()

    def execute(self):
        if not self.jobs:
            return
        # submit jobs
        self.futs = [self.task.remote(job) for job in self.jobs]
        self.jobs.clear()

    @staticmethod
    @ray.remote
    def task(job):
        ret = job.job()
        if isinstance(ret, job_return_type):
            ret.callback(**ret.kwargs)

    def __del__(self):
        [ray.cancel(f) for f in self.futs]

