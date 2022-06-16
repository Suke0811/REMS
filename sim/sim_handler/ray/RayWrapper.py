from sim.sim_handler.ray import autocounter
from sim.sim_handler.ray.SimActor import SimActor
from sim.utils.tictoc import tictoc
from sim.robots.scalear_leg.ScalerManipulatorDef import ScalerManipulator
from sim.robots.bind_robot import bind_robot
import time
import ray

EXCLUDE = ['call_func', 'get_variable', 'set_variable']
def get_methods(instance):
    rets = []
    for attr in dir(instance):
        if callable(getattr(instance, attr)):
            if not '__' in attr and not attr in EXCLUDE:
                rets.append(attr)
    return rets

def get_vars(instance):
    rets = []
    for k,v in vars(instance).items():
        if not '__' in k:
            rets.append(k)
    return rets



class RobotRayWrapper(object):
    def __init__(self, robot, outputs, cache=False):
        self._local_robot = robot
        self._cache = cache
        self._ray_robot = SimActor.options(name=robot.run.name+str(time.time()), max_concurrency=2).remote(robot, outputs)
        self._ray_methods = get_methods(robot)
        self._ray_actor_methods = get_methods(self._ray_robot)
        self._ray_vars = get_vars(robot)
        self.refs = {}
        for m in self._ray_methods:
            self._add_method(m)
        for a_m in self._ray_actor_methods:
            self._add_method(a_m, False)
        for v in self._ray_vars:
            self._add_var(v)


    def _add_method(self, name, robot_method=True):
        self.refs[name] = []
        if robot_method:
            func_str = "self._ray_robot._call_func.remote(name, *args, **kwargs)"
        else:
            func_str = "self._ray_robot."+name+".remote(*args, **kwargs)"
        func = eval('lambda self, name, *args, **kwargs: ' + func_str)    # this is a trick to speed up things

        def method(self, *args, **kwargs):
            #refs = self.refs[name]
            if 'block' in kwargs and not kwargs.pop('block'):
                # if refs:
                #     finished, refs = ray.wait(self.futs, num_returns=len(refs))
                #     self.refs[name] = refs
                #     ret = ray.get(finished[-1])
                # refs.append(func(self, name, *args, **kwargs))
                ret = func(self, name, *args, **kwargs)
            else:
                ret = ray.get(func(self, name, *args, **kwargs))
            return ret

        setattr(self, name, method.__get__(self))

    def _add_var(self, name):   # currently no caching

        def getter(self):
            val = ray.get(self._ray_robot._get_variable.remote(name))
            return val

        def setter(self, val):
            self._ray_robot._set_variable.remote(name, val)

        setattr(RobotRayWrapper, name, property(fget=getter, fset=setter).__get__(self))

        #eval(f'RobotRayWrapper.{name} = property(fget=getter, fset=setter)')
        #setattr(self, name, method.__get__(self))


if __name__ == '__main__':
    from sim.robots.scalear_leg.ScalarHard import ScalerHard
    s = bind_robot(ScalerManipulator,(ScalerHard,'/dev/MOTOR_0', 2))
    ray.init(local_mode=False)
    r = RobotRayWrapper(s, (None,))
    r.set_DT(0.01)
    r.run.DT = 1
    print(r.run.DT)

    N =10000
    st = time.perf_counter()
    for n in range(N):
        r.state.set([1,2,3])
        print(r.state)

    total = time.perf_counter() - st


    st = time.perf_counter()
    for n in range(N):
        s.state.set([1, 2, 3])
        print(s.state)

    total2 = time.perf_counter() - st
    print(total)
    print(total / N)
    print(total2)
    print(total2 / N)
    pass




