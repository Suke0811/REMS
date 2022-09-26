from functools import wraps
from time import perf_counter as time

def tictoc(f):
    @wraps(f)
    def wrap(*args, **kw):
        ts = time()
        result = f(*args, **kw)
        te = time()
        print(f'Function {f.__name__} took {te-ts} seconds')
        return result
    return wrap
