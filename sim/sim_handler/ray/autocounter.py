from functools import wraps

def count_up(f):
    @wraps(f)
    def wrap(self, *args, **kw):
        self.auto_count_check += 1
        result = f(self, *args, **kw)
        return result
    return wrap

def get_count(self):
    return self.auto_count_check

def auto_count(cls):
    orig_init = cls.__init__
    @wraps(cls)
    def __init__(self, *args, **kws):
        orig_init(self, *args, **kws)  # Call the original __init__
        self.auto_count_check = 0

    cls.__init__ = __init__  # Set the class' __init__ to the new one

    for attr in cls.__dict__: # there's propably a better way to do this
        if not attr == '__init__':
            if callable(getattr(cls, attr)):
                setattr(cls, attr, count_up(getattr(cls, attr)))
    setattr(cls, 'get_count', get_count)
    return cls



if __name__ == '__main__':
    @auto_count
    class Foo:
        def foo(self):
            print(self.get_count())

    f = Foo()
    for n in range(100):
        f.foo()

    pass
