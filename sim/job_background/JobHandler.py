from sim.job_background.job_type import job_type
from sim.job_background.job_return_type import job_return_type
from sim.job_background.JobHandlerBase import JobHandlerBase
from concurrent.futures import ThreadPoolExecutor

class JobHandler(JobHandlerBase):
    def __init__(self):
        super().__init__()
        self.executor = ThreadPoolExecutor()

    def find_job(self, jobs):
        if not isinstance(jobs, list):
            jobs = [jobs]
        for l in jobs:
            if isinstance(l, job_type):
                self.jobs.append(l)

    def execute(self):
        if not self.jobs:
            return
        # submit jobs
        futs = [self.executor.submit(job.job) for job in self.jobs]
        # add listener
        for f in futs:
            f.add_done_callback(self.callback)
        self.jobs.clear()

    @staticmethod
    def callback(future):
        """called upon done. call process callback functions"""
        ret = future.result()
        if isinstance(ret, job_return_type):
            ret.callback(**ret.kwargs)


    def __del__(self):
        self.executor.shutdown(cancel_futures=True)
