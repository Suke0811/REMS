from sim.type import DefDict
from sim.robots.RobotBase import RobotBase
from sim.robots.RobotDefBase import RobotDefBase


def bind_robot(robot_def: RobotDefBase, bind: RobotBase, *args):
    class RobotDef(robot_def, bind):
        def __init__(self):
            robot_def.__init__(self)
            bind.__init__(self, *args)
            robot_def.define(self)
    return RobotDef()
