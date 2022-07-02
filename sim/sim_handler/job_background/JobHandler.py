from sim.sim_handler.job_background.job_return_type import job_return_type
from sim.sim_handler.job_background.JobHandlerBase import JobHandlerBase
from concurrent.futures import ThreadPoolExecutor

class JobHandler(JobHandlerBase):
    def __init__(self):
        super().__init__()
        self.executor = ThreadPoolExecutor()

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
        self.executor.shutdown()
