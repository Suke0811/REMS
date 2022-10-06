from rems.typing import DefDict
from rems.typing.std.StdUnit import Pos
from time import perf_counter

st =perf_counter()
d = DefDict(dict(x=float,y=float,z=float))
print(perf_counter()-st)

st =perf_counter()
n = DefDict(dict(x=float,y=float,z=dict(a=32,b=6)))
print(perf_counter()-st)

st =perf_counter()
c = DefDict(dict(x=Pos,y=Pos,z=Pos)).list()
n.set([1,2, [3,4]])
print(perf_counter()-st)
