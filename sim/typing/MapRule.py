from sim.typing import DefDict


class MapRule:
    def __init__(self, origin, func=None, target=None, inv_func=None, with_target=False, to_list=False):
        if origin is None:
            self.origin = None
        else:
            self.origin = DefDict(origin)
        if func is not None and not callable(func):
            raise TypeError(f'func: {func} is not callable')
        self.func = func

        if target is None:
            self.target = None
        else:
            self.target = DefDict(target)
        if inv_func is not None and not callable(inv_func):
            raise TypeError(f'func: {func} is not callable')
        self.inv_func = inv_func
        self.inherit_units = True
        self.to_list = to_list
        self.with_target = with_target

    def map(self, origin, target=None):
        return self._apply_map(self.origin, origin, self.func, self.target, target)

    def inv_map(self, inv_origin, inv_target=None):
        return self._apply_map(self.target, inv_origin, self.inv_func, self.origin, inv_target)


    def _apply_map(self, origin, origin_data, func, target, target_data):
        # if self.target is not None and self.target != name:
        #     return # if not
        if origin is None:
            origin = DefDict(origin_data)
        else:
            if isinstance(origin_data, DefDict) and self.inherit_units:
                for k, o_def in origin_data.DEF.items():
                    if k in origin.keys():
                        origin._definition[k] = [o_def]
            origin.set(origin_data)

        if target is None:
            if target_data is not None:

                target = DefDict(target_data)
        else:
            if isinstance(target_data, DefDict) and self.inherit_units:
                for k, t_def in target_data.DEF.items():
                    if k in target.keys():
                        target._definition[k] = [t_def]

        if func is None:
            if target is None:
                return origin
            else:
                return self._positional_rule(origin, target)
        else:
            if target is not None:
                ret = self._apply_func(origin, func, target)
                if ret is None:
                    return ret
                target.set(ret)
                return target
            else:
                ret = self._apply_func(origin, func, target)
                return ret

    def _apply_func(self, origin, func, target):
        if self.with_target and target is not None:
            if self.to_list:
                ret = func(*origin.list(), *target.list())
            else:
                ret = func(origin, target)
        else:
            if self.to_list:
                ret = func(*origin.list())
            else:
                ret = func(origin)
        return ret

    @staticmethod
    def _positional_rule(origin, target):
        length = min([len(origin), len(target)])
        count = 0

        new_def = {}
        values = []
        for k_o, k_t in zip(origin.keys(), target.keys()):
            if count > length:
                break
            new_def[k_t] = origin._definition[k_o][0]
            values.append(origin._data[k_o][0])
        o = DefDict(new_def).update(values)
        return target.set(o)

    def if_rule_apply(self, keys):
        # true when all bind_from.keys() are in keys
        return all(elem in keys for elem in self.origin.keys())

    def if_rule_out_contains(self, keys):
        # true when all keys are in bind_to.keys()
        if self.target is None:
            return False
        return all(elem in self.target.keys() for elem in keys)
