from sim.robot_def import Manipulator5DoF
from sim.typing.std.StdUnit import Pos, Vel, Acc, Ang, AngVel, AngAcc, UnitType, Percent

from sim.typing import MapRule, DefDict

# sensor names and definitoins

OUTPT = {
"bumper_left": bool, "bumper_right": bool,
    "cliff_left": int, "cliff_front_left": int,
    "cliff_front_right": int, "cliff_right": int
}

ID_LISTs = [2, 1]

SENSOR = {"Webots": OUTPT}


drange =(-3.14, 3.14)
drange_vel = (-1.57, 1.57)
drange_pos = (0, 0.025)
drange_pos_vel = (-10,10)
wb_drive = DefDict({
        "arm1": dict(pos=Ang(drange=drange), vel=AngVel(drange=drange_vel), acc=AngAcc, on=bool),
        "arm2": dict(pos=Ang(drange=drange), vel=AngVel(drange=drange_vel), acc=AngAcc, on=bool),
        "arm3": dict(pos=Ang(drange=drange), vel=AngVel(drange=drange_vel), acc=AngAcc, on=bool),
        "arm4": dict(pos=Ang(drange=drange), vel=AngVel(drange=drange_vel), acc=AngAcc, on=bool),
        "arm5": dict(pos=Ang(drange=drange), vel=AngVel(drange=drange_vel), acc=AngAcc, on=bool),
        "finger1": dict(pos=Pos(drange=drange_pos), vel=Vel(drange=drange_pos_vel), acc=Acc, on=bool),
        "finger2": dict(pos=Pos(drange=drange_pos), vel=Vel(drange=drange_pos_vel), acc=Acc, on=bool),
    })



DRIVE = {
    "Webots": wb_drive,
}


J = dict(pos=Ang(drange=(-3.14, 3.14)), vel=AngVel(drange=(-1.57, 1.57)))

class YoubotArmDef(Manipulator5DoF):
    def __init__(self, *args, **kwargs):
        super().__init__(self, *args, **kwargs)

    def define(self, *args, **kwargs):
        Manipulator5DoF.define(self, joint_units=J, drive_space=DRIVE, sense_space=SENSOR, *args, **kwargs)
        to_position = MapRule(['j.0', 'j.1', 'j.2', 'j.3', 'j.4', 'g.0'],
                              self.set_pos,
                              wb_drive.list_keys(),
                              with_target=True)
        self.drive_space['Webots'].set_rule(to_position)
        self.name = 'youBotArm'

    def set_pos(self, o, t):
        reset = False
        for pos in o.pos().values():
            if pos is not None:
                reset = True
                break
        if reset:
            t.pos().set_positional(o.pos())
        else:
            t.pos().set(t.pos() + o.vel().ndarray() * self.run.DT)
        t.pos().set({'finger2': t.pos().get('finger1')})
        return t
