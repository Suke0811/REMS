
class Operator:
    def __new__(cls, lite_operator=True, *args, **kwargs):
        if lite_operator:
            from rems.LiteOperator import LiteOperator
            return super(Operator, cls).__new__(LiteOperator)
        else:
            from rems.RayOperator import RayOperator
            return super(Operator, cls).__new__(RayOperator)

    def __init__(self, *args, **kwargs):
        print("Initializing MyClass")


