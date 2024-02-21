from rems.sim_handler.job_background import JobHandler as JobHandler


class ProcessActor:
    def __init__(self, process, *args, **kwargs):
        self.process_system = process(*args)
        self.jHandler = JobHandler()

    def init(self, *args, **kwargs):
        self.process_system.init()

    def process(self, t):
        rets = self.process_system.process(t)
        self.jHandler.find_job(rets)
        self.jHandler.execute()

    def get_inpt(self):
        return self.process_system.get_inpt()

