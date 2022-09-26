import ray

@ray.remote
class ActorWrapper:
    def __init__(self, remote_class):
        self.remote_class = remote_class

    def _call_func(self, name, *args, **kwargs):
        return getattr(self.remote_class, name)(*args, **kwargs)

    # for communication
    def _get_variable(self, name):
        return getattr(self.remote_class, name)

    def _set_variable(self, name, val):
        #eval("self.remote_class."+name+"= val")
        setattr(self.remote_class, name, val)

    def get_class(self):
        return self.remote_class



