import numpy as np

from rems.process import ProcessSystem
from rems.sim_handler.job_background.job_type import job_type
from rems.sim_handler.job_background import job_return_type


class TestProcess(ProcessSystem):
    def __init__(self):
        self.count = 0  # counter

    def process(self):
        self.count += 1
        if self.count % 10 == 0: # only submit job every 10 time step
            return job_type(self.job)


    def job(self, **kwargs):
        #time.sleep(1)   # a simple job, just wait 1sec
        for i in range(100000):
            A= np.random.sample(size=(1000,1000))
            B=np.dot(A,A)

        kwargs.update({'count': i})  # add whatever variables you want to return
        return job_return_type(self.done, **kwargs) # pack and go

    def done(self, **kwargs):
        t = kwargs.get("count")  # you can extract variables
        print("done: {}".format(t)) # print to show it is working
