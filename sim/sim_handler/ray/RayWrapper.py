from sim.sim_handler.ray.ActorWrapper import ActorWrapper
import time
import ray

EXCLUDE = ['get_class'] #['call_func', 'get_variable', 'set_variable']
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


class RayWrapper(object):
    def __init__(self, remote_class, name=None):
        self._local_class = remote_class
        if name is None:
            name = 'ray'
        #self._ray_robot = ActorWrapper.options(name=robot.run.name+str(time.time()), max_concurrency=2).remote(robot, outputs)
        self._ray_class = ActorWrapper.options(name=name + str(time.time())).remote(remote_class)
        self._ray_methods = get_methods(remote_class)
        self._ray_actor_methods = get_methods(self._ray_class)
        self._ray_vars = get_vars(remote_class)
        self._refs = {}
        self._not_serializable = True
        for m in self._ray_methods:
            self._add_method(m)
        for a_m in self._ray_actor_methods:
            self._add_method(a_m, False)
        for v in self._ray_vars:
            self._add_var(v)


    def _add_method(self, name, remote_class_method=True):
        if remote_class_method:
            func_str = "self._ray_class._call_func.remote(name, *args, **kwargs)"
        else:
            func_str = "self._ray_class."+name+".remote(*args, **kwargs)"
        func = eval('lambda self, name, *args, **kwargs: ' + func_str)    # this is a trick to speed up things
        self._refs[name] = []

        def method(self, *args, **kwargs):
            ret = None
            ray_ret = self._refs.get(name)
            if 'cache' in kwargs and kwargs.pop('cache'):
                if ray_ret:
                    finished, ray_ret = ray.wait(ray_ret, num_returns=len(ray_ret))
                    if finished:
                        ret = ray.get(finished[-1])
                ray_ret.append(func(self, name, *args, **kwargs))
                self._refs[name] = ray_ret
            elif 'block' in kwargs and not kwargs.pop('block'):
                    func(self, name, *args, **kwargs)
            else:
                ret = ray.get(func(self, name, *args, **kwargs))
            return ret

        setattr(self, name, method.__get__(self))

    def _add_var(self, name):   # currently no caching
        fget = lambda self: self.getter(name)
        fset = lambda self, value: self.setter(name, value)
        setattr(RayWrapper, name, property(fget=fget, fset=fset, doc='test'))

    def setter(self, name, val):
        self._ray_class._set_variable.remote(name, val)

    def getter(self, name):
        val = ray.get(self._ray_class._get_variable.remote(str(name)))
        return val

    def __getattr__(self, name):
        if '_ray_vars' in self.__dict__:
            if not name in self.__dict__['_ray_vars']:
                if name in get_vars(self.get_class()):
                    self._add_var(name)
                    self.__dict__['_ray_vars'].append(name)
                    return getattr(self, name)
        return super(RayWrapper, self).__getattribute__(name)

    def __setattr__(self, name, value):
        if name not in self.__dict__.keys():
            if 'get_class' in self.__dict__ and name in get_vars(self.get_class()):
                self._add_var(name)
                self.__dict__['_ray_vars'].append(name)
        else:
            self.__dict__[name] = value
        return super(RayWrapper, self).__setattr__(name, value)

    def _reset_attr(self):
        for v in self._ray_vars:
            self._add_var(v)

    def get_class(self):
        if self._not_serializable:
            ret = self._local_class
        else:
            ret = ray.get(self._ray_class.get_class.remote())
        return ret



