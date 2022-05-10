from sim.job_background.job_return_type import job_return_type
from sim.job_background.job_type import job_type


class ProcessSystem:
    def process(self):
        """Called every timestep
        :return job_type, job to submit"""
        return [job_type(self.job)]

    def job(self, **kwargs):
        """a job that is done in background
        :return job_return_type: instance to self.done and **kwargs """
        return job_return_type(self.done, **kwargs)

    def done(self, **kwargs):
        """called when the job is done"""
