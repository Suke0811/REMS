import time

from rems.inputs.JoystickInput import JoystickInput
from rems.typing import DefDict, MapRule
from rems.typing.definitions import *


class JoyManipulator(JoystickInput):
    def __init__(self):
        super(JoyManipulator, self).__init__()
        self.dt = 0.1
        self.prev_time = 0.0
        self.vel = [0.1, 0.1, -0.05]
        self.home = DefDict((POS_3D, VEL_POS_3D)).set(dict(x=0.05, z=-.3))
        xy_stick = MapRule(['X', 'Y'],
                           lambda *args: [x*v for x, v in zip(args, self.vel[0:len(args)])],
                           ['d_y', 'd_x'], to_list=True)
        # if None is returned in func, it has no effect
        a_button = MapRule('A', lambda a: self.home if a else None, (POS_3D, VEL_POS_3D), to_list=True)
        c_stick = MapRule('C_Stick_X', lambda z: self.vel[2]*z, 'd_z', to_list=True)
        #twist = BindRule('Twist', lambda twist )
        self.sum_rules = [xy_stick, c_stick]
        self.equal_rule = [a_button]
        
        self.calc_pos = MapRule(VEL_POS_3D,
                            lambda *args: [self.dt*x for x in args],
                            POS_3D, to_list=True)

    def init(self, input_def=None):
        super().init(input_def)
        if input_def is None:
            self.input_def = self.home.clone()
        else:
            self.input_def = input_def
            self.input_def.set(self.home)


    def get_inputs(self, inpt: DefDict=None, timestamp=None):
        joy_inpt = super().get_inputs()
        if timestamp is not None:
            self.dt = self.prev_time - timestamp
            self.prev_time = timestamp

        for r in self.sum_rules:
            self.input_def += r.map(joy_inpt)

        self.input_def += self.calc_pos.map(self.input_def)

        for r in self.equal_rule:
            self.input_def.set(r.map(joy_inpt))

        if inpt is None:
            inpt = self.input_def.clone()
        inpt.set(self.input_def)
        self.input_def.filter(VEL_POS_3D).set([0, 0, 0])
        return inpt



if __name__ == '__main__':
    j = JoyManipulator()
    j.init()
    while True:
        print(j.get_inputs())
        time.sleep(1)
