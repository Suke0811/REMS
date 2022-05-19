from sim.type import DefDict
from sim.robots.RobotBase import RobotBase
from sim.robots.RobotDefBase import RobotDefBase


def bind_robot(robot_def: RobotDefBase, bind: RobotBase):
    class RobotDef(RobotDefBase, bind): pass
    return RobotDef()
