from rems.sim_handler.job_background import job_return_type
from rems.sim_handler.job_background.job_type import job_type


class ProcessSystem:
    def __init__(self, *args, **kwargs):
        self.robot = None
        self.to_thread = False
        self.dt = 0.01

    def init(self, robot=None, *args, **kwargs):
        self.robot = robot

    def process(self, t, *args, **kwargs):
        """Called every timestep
        :return job_type, job to submit"""
        return [job_type(self.job)]

    def job(self, *args, **kwargs):
        """a job that is done in background
        :return job_return_type: instance to self.done and **kwargs """
        return job_return_type(self.done, **kwargs)

    def done(self, *args, **kwargs):
        """called when the job is done"""

    def process_callback(self, *args, **kwargs):
        """
        callback for SimRay
        """
