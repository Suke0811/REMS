from sim.type import DefDict, DefBind, DefBindRule
from sim.type.definitions import *
from typing import Any
from sim.type.definitions import *
from scipy.spatial.transform import Rotation as R


#rotation conversion rule
# Euler to Rotation vector

EULER_2_ROT_VECTOR = DefBindRule(EULER_3D, lambda a,b,c: R.from_euler('xyz',[a,b,c]).as_rotvec(), ROT_VECTOR)

d=DefBind(dict(a=EULER_2_ROT_VECTOR,b=EULER_2_ROT_VECTOR,c=EULER_2_ROT_VECTOR))
pass
