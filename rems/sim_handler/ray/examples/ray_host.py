import time

import ray
from ray.util.scheduling_strategies import  NodeAffinitySchedulingStrategy

from rems.device.Dynamixel import Dynamixel
from rems.inputs import JoystickInput




# ray.init(runtime_env={"working_dir": '..', "excludes": ["video"],"pip":"../requirements.txt",
#                       "env_vars": {"RAY_RUNTIME_ENV_SKIP_LOCAL_GC": "1"}})
ray.init(runtime_env={"working_dir": '..', "excludes": ["video", "out", ".git"],
                      "pip":"../requirements.txt",
                      "env_vars": {"DISPLAY": ":0"}})
print(ray.cluster_resources())


@ray.remote
class ActorOnNode:
    def __init__(self):
        print('hi')
        print('there')
        self.joy = JoystickInput()


    def busy_looping(self):
        t = time.perf_counter()
        while True:
            if time.perf_counter() - t >= 0.1:
                print('init')
                break

    def dynamixel(self):
        print('dynamixel')
        self.dy = Dynamixel([0, 1])
        self.dy.init()
        self.dy.open()
        self.dy.enable(enable=True)

    def joystick(self):
        return self.joy.get_inputs()






nodes = ray.nodes()
node = nodes[0].get('NodeID')
for n in nodes:
    name = n.get('NodeManagerAddress')
    if name == '192.168.99.40':
        node = n.get('NodeID')
        break

a = ActorOnNode.options(scheduling_strategy=NodeAffinitySchedulingStrategy(node_id=node, soft=False)).remote()
ray.get(a.busy_looping.remote())

ref = []
total = time.perf_counter()
for n in range(100):
    t = time.perf_counter()
    if ref:
        finished, ref = ray.wait(ref, num_returns=1, )
        ret = ray.get(finished)
        print(ret)
    ref.append(a.joystick.remote())
    print(time.perf_counter()-t)

print((time.perf_counter()-total)/100.0)
pass
