from sim.type import DefDict
from sim.robots.RobotBase import RobotBase
from typing import Any

QUEUE_IN=dict(inpt=DefDict, t_init=float, dt=float)
QUEUE_OUT=dict(robot=Any, outpt=Any)
