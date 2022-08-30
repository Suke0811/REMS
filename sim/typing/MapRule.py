from sim.typing import DefDict


class MapRule:
    def __init__(self, origin, func=None, target=None, inv_func=None, with_target=False):
        self.origin = DefDict(origin)
        if not callable(func):
            raise TypeError('func: {func} is not callable')
        self.func = func
        self.target = DefDict(target)
        if not callable(inv_func):
            raise TypeError('func: {func} is not callable')
        self.inv_func = inv_func

        self.with_target = with_target

    def map(self, origin, target=None):
        return self._apply_map(self.origin, origin, self.func, self.target, target)

    def inv_map(self, inv_origin, inv_target):
        return self._apply_map(self.target, inv_origin, self.inv_func, self.origin, inv_target)

    def _apply_map(self, origin, origin_data, func, target, target_data):
        # if self.target is not None and self.target != name:
        #     return # if not
        if origin is None:
            origin = origin_data
        else:
            origin.set(origin_data)

        if target is None:
            if target_data is not None:
                target = DefDict(target_data)

        if func is None:
            if target is None:
                return origin
            else:
                return target.set(origin.list())
        else:
            if target is not None:
                if self.with_target:
                    ret = func(*origin.list())
                else:
                    ret = func(*origin.list(), *target.list())
                if ret is None:
                    return ret
                target.set(ret)
                return target.dict()
            else:
                if self.with_target:
                    ret = func(*origin.list())
                else:
                    ret = func(*origin.list())
                return ret

    def if_rule_apply(self, keys):
        # true when all bind_from.keys() are in keys
        return all(elem in keys for elem in self.origin.keys())

    def if_rule_out_contains(self, keys):
        # true when all keys are in bind_to.keys()
        if self.target is None:
            return False
        return all(elem in self.target.keys() for elem in keys)
