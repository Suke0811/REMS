

import time
import ray
import numpy as np

@ray.remote
class ray_test:
    def __init__(self, st):
        self.st = st    # start time

    def show(self, val):    # just print val and time spent
        print(f"n: {val}, t: {time.perf_counter()-self.st}")

st = time.perf_counter()    # start time
r = ray_test.remote(st)     # instance

N = 100
val = np.array([0])
for n in range(N):  # call show() for N times
    val += 1
    ret = r.show.remote(n)
    ray.get(ret)
    #time.sleep(0.1)
