import ray
from ray.util.queue import Queue, Empty
import time
from sim.type import DefDict
from sim.type.definitions import *
from sim.robots.scalear_leg.ScalerManipulatorDef import ScalerManipulator
from sim.robots.scalear_leg.ScalarHard import ScalerHard
from sim.robots.bind_robot import bind_robot

@ray.remote
class queue_test:
    def __init__(self, q_in, q_out):
        self.q_in = q_in
        self.q_out = q_out

    def main(self):
        while True:
            robot = self.q_in.get()
            print(robot.inpt.data)
#            robot.state.data['y'] += 1
            self.q_out.put(robot)

q_in = Queue()
q_out = Queue()
a = queue_test.remote(q_in,q_out)
N = 10000
a.main.remote()
d = DefDict(POS_3D)

scaler = ref_robot = bind_robot(ScalerManipulator, ScalerHard, '/dev/ttyUSB0')
scaler.define()
sca = scaler

st = time.perf_counter()
for n in range(N):
    scaler.inpt.data = [n]
    q_in.put(scaler)

    scaler = q_out.get()
    sca = scaler
    print(scaler.state.data)

print((time.perf_counter()-st)/N)
