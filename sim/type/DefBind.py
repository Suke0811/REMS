from sim.type import DefDict, DefDictData
from typing import Any


class DefBindRule:
    def __init__(self, bind_from, bind_func=None, bind_to=None, dtype=Any):
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
            self.bind_to = DefDict(bind_to, type_=dtype)
        if bind_from is None:
            self.bind_from = None
        else:
            self.bind_from = DefDict(bind_from, type_=dtype)
        self.bind_func = bind_func
        self.type_ = dtype

    def bind(self, data, bind_to=None):
        if self.bind_from is None:
            return  # None rule do nothing

        self.bind_from.data = data
        if self.bind_to is None:
            if bind_to is not None:
                self.bind_to = DefDict(bind_to, Any)
        if self.bind_func is None:
            self.bind_to.data = self.bind_from.data.as_list()
        else:
            if self.bind_to is not None:
                self.bind_to.data = self.bind_func(*self.bind_from.data.as_list())
                return self.bind_to.data
            else:
                return self.bind_func(*self.bind_from.data.as_list())


if __name__ == '__main__':
    # create a dict of binding rule
    # key is definition of the input
    # bind_from indicates which named variables should be used to calculate the values
    # bind_func (optional) is a function used to calculate the value: bind_to = func(bind_from). Default is 1-1 mapping
    # bind_to (optional) indicates to which the calculated value should be applied. Default to key.
    # bind_to
    # key
    #  └─ bind from
    #  └─ bind function
    #  └─ bind to
    #  └─ type
    #this defines that the 'k' is calculated by 'b' and 'c' using lambda function.
    rule = DefBindRule(['b', 'c'], lambda b, c: 2 * b + c)
    a = {'t': rule}
    # then formulate binding
    #b = DefBind(a)
    # say we have data input
    inpt = {'a':0.0, 'b':2.0, 'c':6.0}
    # bind(inpt) will apply the binding rule
