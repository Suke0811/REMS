from sim.job_background.job_type import job_type

class JobHandlerBase:
    def __init__(self):
        self.jobs =[]

    def find_job(self, jobs):
        if not isinstance(jobs, list):
            jobs = [jobs]
        for l in jobs:
            if isinstance(l, job_type):
                self.jobs.append(l)

    def execute(self):
        pass

