from sim.type.DefDict import DefDict
from typing import Any

INPT='inpt'
T_INIT='t_init'
DT='dt'
QUEUE_IN={INPT: DefDict, T_INIT:float, DT:float}
ROBOT='robot'
OUTPT='outpt'
QUEUE_OUT={ROBOT: Any, OUTPT: Any}

