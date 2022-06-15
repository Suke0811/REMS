import ray


@ray.remote(num_cpus=1)
class ProcessActor:
    def __init__(self, process):
        self.process = process
        # self.jHandler = JobHandler()

    def process(self):
        rets = self.process.process()
        # self.jHandler.find_job(rets)
        # self.jHandler.execute()

