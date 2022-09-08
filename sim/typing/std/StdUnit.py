import numpy as np
from sim.typing import DefDict, UnitType, MapRule
from typing import Union
from scipy.spatial.transform import Rotation as R


"""
Here are predefined physical quantities that are predefined. You can define your own by extending UnitType.
You can also change the settings when you create a instance as follow:
dict(x=Pos(unit='cm', drange=('-1km', '1000m'), dtype=int), y=Pos())
Note that if you do:
Pos_cm = Pos(unit='cm', drange=('-1km', '1000m'), dtype=int)
dict(x=Pos_cm, y=Pos_cm)
changing values in Pos_cm will affect the both
"""


class Example(UnitType):
    # unit
    default_unit = 'dimensionless'
    # default value for this unit, if none then we do dtype(), i.e. float() = 0.0
    default_value = None
    # data type
    default_dtype = float
    # dimension i.e. 1D array of 3 columns -> 3 or 2D array 2 by 3 -> (2, 3)
    default_dim: Union[int, tuple[int, int]] = 1
    # data range for this unit
    default_drange: tuple[(Union[int, float, str], Union[int, float, str])] = (float('-inf'), float('inf'))
    # data mapping to let you create your own unit conversion
    # unit='count'; drange=(0, 4095); drange_map=('0.01m', '1m')
    # This indicates 0 count corresponds to 0.01m and 4096 to 1m and inbetween is linear interpolation
    # drange_map=('-1m', func, '1m') and takes func(val, sv, lv). this let you define custom interpolation
    # you can also have more points. If no func is specified, then it'll be linear interpolation
    # drange=(0, 1024, 2048, 3072 ,4096); drange_map=('10mm', '0.1m', '0.5m', func, '0.75m', ,'1m')
    default_drange_map: tuple[(Union[int, float, str], Union[int, float, str])] = None
    # data scale. (-1, 1) -> -100% to 100%. (0, 1) -> 0% to 100%
    defualt_drange_scale: tuple[(Union[int, float], Union[int, float])] = (-1, 1)


class Time(UnitType):
    default_unit = 's'
    default_drange = (0, float('inf'))
    defualt_drange_scale = (0, 1)

class Pos(UnitType):
    default_unit = 'm'

class Vel(UnitType):
    default_unit = 'm/s'

class Acc(UnitType):
    default_unit = 'm/s**2'

class Ang(UnitType):
    default_unit = 'rad'

class AngVel(UnitType):
    default_unit = 'rad/s'

class AngAcc(UnitType):
    default_unit = 'rad/s**2'

class Mass(UnitType):
    default_unit = 'kg'
    default_drange = (0, float('inf'))
    defualt_drange_scale = (0, 1)

class Percent(UnitType):
    default_unit = 'percent'
    defualt_drange_scale = (0, 1)

class Count(UnitType):
    default_unit = 'count'
    default_dtype = int
    default_drange = (0, 4095)
    defualt_drange_scale = (0, 1)

class CountSensor(UnitType):
    default_unit = 'count'
    default_dtype = int
    default_drange = (0, 4095)
    default_drange_map = ('0.1m', '1m')
    defualt_drange_scale = (0, 1)

class VelCreate(UnitType):
    default_unit = 'rad/s'
    default_drange = (-5,5)

class CountVel(UnitType):
    default_unit = 'count'
    default_dtype = int
    default_drange = (-500, 500)
    default_drange_map = ('-5rad/s', '5rad/s')
    defualt_drange_scale = (0, 1)

from time import perf_counter

v = VelCreate()
p = Percent()
c = CountVel()
v.to(5, p)
v.to(5, c)
c.from_count(500)
d = DefDict(dict(x=Pos, y=Pos)).set([1,2])
nd = DefDict(dict(x=Pos(unit='mm'), y=Pos(unit='cm'))).update([100,10])

dv = DefDict({'wh.r': VelCreate, 'wh.l': VelCreate}).set([1,2])
dc = DefDict({'wh.r': CountVel(),'wh.l': CountVel()})
dc.clone()
print(d+nd)
dc.set(dv)

pass
# Rotation
# if you do
# dict(qw=float, qx=float, qy=float, qz=float)
# this will be automatically set to
# dict(qw=Default(dtype=float), qx=Default(dtype=float), qy=Default(dtype=float), qz=Default(dtype=float))
# so unitless (there will be no unit dimension check)
#
#
# quat = dict(definition=dict(qw=float, qx=float, qy=float, qz=float), name='quat')
# rot3d = dict(definition=dict(r11=float, r12=float, r13=float,
#                                  r21=float, r22=float, r23=float,
#                                  r31=float, r32=float, r33=float,), name='rot3d', shape=(3, 3))
# rot2d = dict(definition=dict(r11=float, r12=float,
#                                  r21=float, r22=float,), name='rot2d', shape=(2, 2))
# euler = dict(definition=dict(th_x=Ang, th_y=Ang, th_z=Ang, _coord=str), name='euler')
# ax_ang = dict(definition=dict(rx=Ang, ry=Ang, rz=Ang), name='ax_ang')
# ax_ang4d = dict(definition=dict(ux=float, uy=float, uz=float, alpha=Ang), name='ax_ang4d')
#
# def from_ax_ang4d(ax_ang4d):
#     ax_angle = ax_ang4d[0:3] * ax_ang4d[-1]
#     return R.from_rotvec(ax_angle)
#
# def as_ax_ang_4d(r):
#     ax_angle = r.as_rotvec()
#     norm = np.linalg.norm(ax_angle)
#     u_vect = ax_angle / norm
#     return np.append(u_vect, norm)
#
# conversion_rules = [
#     # from or to quat
#     MapRule(quat, lambda quat: R.from_quat(quat.ndarray()).as_matrix(), rot3d,
#                   lambda rot3d: R.from_matrix((rot3d.ndarray())).as_quat()),
#     MapRule(quat, lambda quat: R.from_quat(quat.ndarray()).as_rotvec(), ax_ang,
#                   lambda ax_ang: R.from_rotvec((ax_ang.ndarray())).as_quat()),
#     MapRule(quat, lambda quat: as_ax_ang_4d(R.from_quat(quat.ndarray())), ax_ang4d,
#                   lambda ax_ang4d: from_ax_ang4d(ax_ang4d.ndarray()).as_quat()),
#     MapRule(quat, lambda quat, euler: R.from_quat(quat.ndarray()).as_euler(euler.get('_coord')), euler,
#                   lambda euler: R.from_euler(euler.ndarray(), euler.get('_coord')).as_quat(), with_target=True),
#     # from or to ax_ang
#     MapRule(ax_ang, lambda ax_ang: R.from_rotvec(ax_ang.ndarray()).as_matrix(), rot3d,
#                     lambda rot3d: R.from_matrix((rot3d.ndarray())).as_rotvec()),
#     MapRule(ax_ang, lambda ax_ang, euler: R.from_rotvec(ax_ang.ndarray()).as_euler(euler.get('_coord')), euler,
#                     lambda euler: R.from_euler(euler.ndarray(), euler.get('_coord')).as_rotvec(), with_target=True),
#     MapRule(ax_ang, lambda ax_ang: as_ax_ang_4d(R.from_rotvec(ax_ang.ndarray())), ax_ang4d,
#                     lambda ax_ang4d: from_ax_ang4d((ax_ang4d.ndarray())).as_rotvec()),
#     # from or to rot 3d
#     MapRule(rot3d, lambda rot3d, euler: R.from_matrix(rot3d.ndarray()).as_euler(euler.get('_coord')), euler,
#                    lambda euler: R.from_euler(euler.ndarray(), euler.get('_coord')).as_rotvec(), with_target=True),
#     MapRule(rot3d, lambda rot3d: as_ax_ang_4d(R.from_matrix(rot3d.ndarray())), ax_ang4d,
#                    lambda ax_ang4d: from_ax_ang4d((ax_ang4d.ndarray())).as_matrix()),
#     # ax_ang4d
#     MapRule(ax_ang4d, lambda ax_ang4d, euler: from_ax_ang4d(ax_ang4d.ndarray()).as_euler(euler.get('_coord')), euler,
#                       lambda euler: as_ax_ang_4d(R.from_euler(euler.ndarray(), euler.get('_coord'))), with_target=True),
# ]
#
#
# class Quaternion(UnitType):
#     default_unit = 'quat'
#     default_dtype = DefDict(**quat)
#     default_dim = 4
#     # default_conversion_rules = conversion_rules
#
#
# class Rotation3DMatix(UnitType):
#     default_unit = 'rot3d'
#     default_dtype = DefDict(**rot3d)
#     default_value = np.eye(3) # identy matrix. Under the hood, it does DefDict.set(default_value)
#     default_dim = (3, 3)
#    # default_conversion_rules = conversion_rules
#
# class Rotation2DMatix(UnitType):
#     default_unit = 'rot2d'
#     default_dtype = DefDict(**rot2d)
#     default_value = np.eye(2) # identy matrix. Under the hood, it does DefDict.set(default_value)
#     default_dim = (2, 2)
#
# class Euler(UnitType):
#     default_unit = 'euler'
#     # _coord is a protected key (you don't see unless you implicitly access it)
#     default_dtype = DefDict(**euler)
#     default_value = [0, 0, 0, 'xyz']
#     default_dim = 3
#     default_conversion_rules = conversion_rules
#
# class AxisAngle(UnitType):
#     """
#     This is a 3 element version of the Axis-Angle representation.
#     The first three elements are unit vector times angle (hence dimension is angle)
#     """
#     default_unit = 'ax_ang'
#     default_dtype = DefDict(**ax_ang)
#     default_value = [0, 0, 0] # 0 rotation around z
#     default_dim = 3
#     default_conversion_rules = conversion_rules
#
# class AxisAngle4D(UnitType):
#     """
#     This is a 4 element version of the Axis-Angle representation.
#     The first three elements are unit vector and the 4th element is angle
#     """
#     default_unit = 'ax_ang4d'
#     default_dtype = DefDict(**ax_ang4d)
#     default_value = [0, 0, 1, 0]
#     default_dim = 4
#     default_conversion_rules = conversion_rules
#
#
# if __name__ == '__main__':
#     import sys
#     #sys.setrecursionlimit(30)
#     import time
#     # st  = time.perf_counter()
#     # p = DefDict(dict(x=Pos, quat=Quaternion))
#     # print(time.perf_counter()-st)
#     # p.set(dict(x=Pos(unit='cm', data=10)))
#     # print(time.perf_counter()-st)
#     # p.set(dict(quat=[1,0,0,0]))
#     # print(time.perf_counter()-st)
#
#
#     import unyt
#     st = time.perf_counter()
#     u = unyt.unyt_quantity.from_string('0.1m')
#     print('unyt',time.perf_counter()-st)
#     st = time.perf_counter()
#     d = DefDict(dict(x=float,y=float,z=float))
#     print(time.perf_counter()-st)
#     # d.format(dict(x=0, y=0)).ndarray()
#     # print(time.perf_counter()-st)
#     st = time.perf_counter()
#     r = DefDict(dict(x=Pos, rot3d=Rotation3DMatix))
#     print(time.perf_counter()-st)
#     st = time.perf_counter()
#     q=Quaternion()
#     print('unyt',time.perf_counter()-st)
#     q.to(10)
#     print('unyt',time.perf_counter()-st)
#     pass
