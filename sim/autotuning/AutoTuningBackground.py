from sim.controllers import TuningSystem
from sim.job_background.job_type import job_type
from sim.job_background.job_return_type import job_return_type

HORIZON = 10

class AutoTuning(TuningSystem):
    def __init__(self, target, ref, test_case):
        super().__init__(target_robot=target, reference_robot=ref)
        self.count = 0  # counter

    def process(self):
        self.count += 1
        if self.count % 10 == 0: # only submit job every 10 time step
            return job_type(self.job)

    def job(self, **kwargs):
        #time.sleep(10)
        return job_return_type(self.done, **kwargs) # pack and go

    def done(self, **kwargs):
        t = kwargs.get("count")  # you can extract variables
        print("done: {}".format(t)) # print to show it is working
