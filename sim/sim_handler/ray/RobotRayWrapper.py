from sim.sim_handler.ray import autocounter
from sim.sim_handler.ray.SimActor import SimActor
import time
import ray
TIMEOUT = 0.0005
EXCLUDE = [] #['call_func', 'get_variable', 'set_variable']
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
        self._not_serializable = True
        #self._ray_robot = SimActor.options(name=robot.run.name+str(time.time()), max_concurrency=2).remote(robot, outputs)
        self._ray_robot = SimActor.options(name=robot.run.name + str(time.time())).remote(robot, outputs)
        self._ray_methods = get_methods(robot)
        self._ray_actor_methods = get_methods(self._ray_robot)
        self._ray_vars = get_vars(robot)
        self._refs = {}
        for m in self._ray_methods:
            self._add_method(m)
        for a_m in self._ray_actor_methods:
            self._add_method(a_m, False)
        for v in self._ray_vars:
            self._add_var(v)

    def _add_method(self, name, robot_method=True):
        if robot_method:
            func_str = "self._ray_robot._call_func.remote(name, *args, **kwargs)"
        else:
            func_str = "self._ray_robot."+name+".remote(*args, **kwargs)"
        func = eval('lambda self, name, *args, **kwargs: ' + func_str)    # this is a trick to speed up things

        self._refs[name] = []
        def method(self, *args, **kwargs):
            ret = None
            ray_ret = self._refs.get(name)
            if 'cache' in kwargs and kwargs.pop('cache'):
                if ray_ret:
                    finished, ray_ret = ray.wait(ray_ret, num_returns=len(ray_ret), timeout=TIMEOUT)
                    if finished:
                        ret = ray.get(finished[-1])
                ray_ret.append(func(self, name, *args, **kwargs))
                self._refs[name] = ray_ret
            elif 'block' in kwargs and not kwargs.pop('block'):
                ret = func(self, name, *args, **kwargs)
            else:
                ret = ray.get(func(self, name, *args, **kwargs))
            return ret

        setattr(self, name, method.__get__(self))

    def _add_var(self, name):   # currently no caching
        fget = lambda self: self.getter(name)
        fset = lambda self, value: self.setter(name, value)
        setattr(RobotRayWrapper, name, property(fget=fget, fset=fset, doc='test'))

    def setter(self, name, val):
        self._ray_robot._set_variable.remote(name, val)

    def getter(self, name, *args, **kwargs):
        self._refs[name] = []
        ret = None
        ray_ret = self._refs.get(name)
        if 'cache' in kwargs and kwargs.pop('cache'):
            if ray_ret:
                finished, ray_ret = ray.wait(ray_ret, num_returns=1, timeout=TIMEOUT)
                if finished:
                    ret = ray.get(finished[-1])
            ray_ret.append(self._ray_robot._get_variable.remote(str(name)))
            self._refs[name] = ray_ret
        else:
            ret = ray.get(self._ray_robot._get_variable.remote(str(name)))
        return ret

    def __getattr__(self, name):
        if '_ray_vars' in self.__dict__:
            if not name in self.__dict__['_ray_vars']:
                if name in get_vars(self.get_robot()):
                    self._add_var(name)
                    self.__dict__['_ray_vars'].append(name)
                    return getattr(self, name)
        return super(RobotRayWrapper, self).__getattribute__(name)

    def __setattr__(self, name, value):
        if name not in self.__dict__.keys():
            if 'get_robot' in self.__dict__ and name in get_vars(self.get_robot()):
                self._add_var(name)
                self.__dict__['_ray_vars'].append(name)
        else:
            self.__dict__[name] = value
        return super(RobotRayWrapper, self).__setattr__(name, value)

    def get_robot(self):
        if self._not_serializable:
            ret = self._local_robot
        else:
            ret = ray.get(self._ray_class.get_robot.remote())
        return ret

    def _reset_attr(self):
        for v in self._ray_vars:
            self._add_var(v)



