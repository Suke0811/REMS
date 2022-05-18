from sim.type import DefBind
from sim.type import DefBindRule as Rule


# key binding rule
def key_func(up, down, left, right):
    if up and left:
        ret = [0.5, 1.0]
    elif up and right:
        ret = [1.0, 0.5]
    elif down and left:
        ret = [-0.5, -1.0]
    elif down and right:
        ret = [-1.0, -0.5]
    elif down and up:
        ret = [-1.0, 1.0]
    elif left and right:
        ret = [1.0, -1.0]
    elif up:
        ret = [1.0, 1.0]
    elif down:
        ret = [-1.0, -1.0]
    elif left:
        ret = [0.0, 1.0]
    elif right:
        ret = [1.0, 0.0]
    else:
        ret = [0.0, 0.0]
    return ret

# normal case (Recommended)
# the function can be def... or lambda function
# they only depend on the arguments.
# arguments are not persistent, meaning they are only function of current button states. i.e. pressed or released
keyboard_rule = dict(
    j1=Rule(['up', 'down', 'left', 'right'], key_func, ['j1', 'j2'], float()),
    j2=Rule(None)   # j1 rule provides j2 values, so None rule
)
keyboard_bind = DefBind(keyboard_rule)


axis = ['STICK_LEFT_X', 'STICK_LEFT_Y', 'STICK_RIGHT_X', 'STICK_RIGHT_Y', 'THROTTLE_L', 'THROTTLE_R']

def axis_func(LX, LY, RX, RY, LT, RT):
    pass

joystick_rule = dict(
    j1=Rule(['up', 'down', 'left', 'right'], axis_func, ['j1', 'j2'], float()),
    j2=Rule(None)   # j1 rule provides j2 values, so None rule
)





if __name__ == '__main__':
    class test:
        def __init__(self):
            self.state = False

        def key_func(self, up, down, left, right):
            if up and self.state:
                self.state = False
                return [0.5, 1.0]
            if up and not self.state:
                self.state = True
                return [0, -1]
    test_in = {'up': True, 'down': False, 'left': False, 'right': False}
    print(keyboard_bind.bind(test_in))

    # A test case using a class with internal variables.
    # this could be useful if you like to do persistent actions with button
    # i.e. reverse sign when a certain key is pressed.
    t = test()
    keyboard_rule_class = dict(
        j1=Rule(['up', 'down', 'left', 'right'], t.key_func, ['j1', 'j2'], float()),
        j2=Rule(None)  # j1 rule provides j2 values, so None rule
    )

    # Each time 'up' is pressed, it changes the state even with the same inputs
    keyboard_bind = DefBind(keyboard_rule_class)
    print(keyboard_bind.bind(test_in))
    print(keyboard_bind.bind(test_in))
    print(keyboard_bind.bind(test_in))
