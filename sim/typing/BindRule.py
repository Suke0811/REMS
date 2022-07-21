from sim.typing import DefDict
from typing import Any


class BindRule:
    def __init__(self, bind_from, bind_func=None, bind_to=None, inv_bind_func=None,dtype=Any):
        """
        Create a dict of binding rule
        key is definition of the input
        :param bind_from indicates which named variables should be used to calculate the values
        :param bind_func (optional) is a function used to calculate the value: bind_to = func(bind_from). Default is 1-1 mapping
        :param bind_to (optional) indicates to which the calculated value should be applied. Default to key.
        :param dtype (optional) to define the value type. Default to float
        key
         └─ bind from
         └─ bind function
         └─ bind to
         └─ type
        """
        if bind_to is None:
            self.bind_to = None
        else:
            self.bind_to = DefDict(bind_to, dtype=dtype)
        if bind_from is None:
            self.bind_from = None
        else:
            self.bind_from = DefDict(bind_from, dtype=dtype)
        self.bind_func = bind_func
        self.inv_bind_func = inv_bind_func
        self.type_ = dtype

    def bind(self, data, bind_to=None):
        if self.bind_from is None:
            bind_from = data
        else:
            bind_from = self.bind_from.set(data).list()

        if self.bind_to is None:
            if bind_to is not None:
                self.bind_to = DefDict(bind_to, Any)

        if self.bind_func is None:
            if self.bind_to is None:
                return bind_from
            else:
                return self.bind_to.set(bind_from)
        else:
            if self.bind_to is not None:

                ret = self.bind_func(*bind_from)
                if ret is None:
                    return ret
                self.bind_to.set(ret)
                return self.bind_to.dict()
            else:
                return self.bind_func(*bind_from)

    def inv_bind(self, data, bind_from=None):
        if self.bind_to is None:
            return self.inv_bind_func(*data)
        self.bind_to.set(data)
        if self.bind_from is None:
            if bind_from is not None:
                self.bind_from = DefDict(bind_from, Any)
        if self.inv_bind_func is None:
            self.bind_from.set(self.bind_to.list())
        else:

            if self.bind_from is not None:
                self.bind_from.set(self.inv_bind_func(*self.bind_to.list()))
                return self.bind_from.dict()
            else:
                return self.inv_bind_func(*self.bind_to.list())
