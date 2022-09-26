from rems.typing import DefDict
from rems.robots.RobotBase import RobotBase
from rems.robots.RobotDefBase import RobotDefBase


def bind_robot(robot_def, bind):
    """
    Inherit a class at run time
    :param robot_def: subclass of RobotDefBase
    :param bind: subclass of RobotBase
    :param args: args for robot base
    :return: return instance of the robot
    """
    # if they are tuple, first one is the robot and the rest is arguments
    robot_def_args = tuple()
    bind_args = tuple()

    if isinstance(robot_def, tuple):
        robot_def, *robot_def_args = robot_def

    if isinstance(bind, tuple):
        bind, *bind_args = bind

    if robot_def is None: # if definition is none, then initiate without definition
        robot = bind(*bind_args)
        robot.define()
        return robot

    class RobotDef(robot_def, bind):
        def __init__(self):
            robot_def.__init__(self, *robot_def_args)
            bind.__init__(self, *bind_args)
            # call define
            robot_def.define(self)
    return RobotDef()
