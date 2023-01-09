import numpy as np
from rems.typing import DefDict, UnitType, MapRule
from typing import Union
from scipy.spatial.transform import Rotation as R
from rems.typing.std.StdUnit import Ang, Pos, Float

# Rotation
# if you do
dict(qw=float, qx=float, qy=float, qz=float)
# this will be automatically set to
dict(qw=Float(drange=(-1, 1)), qx=Float(drange=(0, 1)), qy=Float(drange=(0, 1)), qz=Float(drange=(0, 1)))
# so unitless (there will be no unit dimension check)


quat = dict(definition=dict(qw=float, qx=float, qy=float, qz=float), name='quat')
rot3d = dict(definition=dict(r11=float, r12=float, r13=float,
                                 r21=float, r22=float, r23=float,
                                 r31=float, r32=float, r33=float,), name='rot3d', shape=(3, 3))
rot2d = dict(definition=dict(r11=float, r12=float,
                                 r21=float, r22=float,), name='rot2d', shape=(2, 2))
euler = dict(definition=dict(th_x=Ang, th_y=Ang, th_z=Ang, _coord=str), name='euler')
ax_ang = dict(definition=dict(rx=Ang, ry=Ang, rz=Ang), name='ax_ang')
ax_ang4d = dict(definition=dict(ux=float, uy=float, uz=float, alpha=Ang), name='ax_ang4d')

def from_ax_ang4d(ax_ang4d):
    ax_angle = ax_ang4d[0:3] * ax_ang4d[-1]
    return R.from_rotvec(ax_angle)

def as_ax_ang_4d(r):
    ax_angle = r.as_rotvec()
    norm = np.linalg.norm(ax_angle)
    u_vect = ax_angle / norm
    return np.append(u_vect, norm)

conversion_rules = [
    # from or to quat
    MapRule(quat, lambda quat: R.from_quat(quat.ndarray()).as_matrix(), rot3d,
                  lambda rot3d: R.from_matrix((rot3d.ndarray())).as_quat()),
    MapRule(quat, lambda quat: R.from_quat(quat.ndarray()).as_rotvec(), ax_ang,
                  lambda ax_ang: R.from_rotvec((ax_ang.ndarray())).as_quat()),
    MapRule(quat, lambda quat: as_ax_ang_4d(R.from_quat(quat.ndarray())), ax_ang4d,
                  lambda ax_ang4d: from_ax_ang4d(ax_ang4d.ndarray()).as_quat()),
    MapRule(quat, lambda quat, euler: R.from_quat(quat.ndarray()).as_euler(euler.get('_coord')), euler,
                  lambda euler: R.from_euler(euler.ndarray(), euler.get('_coord')).as_quat(), with_target=True),
    # from or to ax_ang
    MapRule(ax_ang, lambda ax_ang: R.from_rotvec(ax_ang.ndarray()).as_matrix(), rot3d,
                    lambda rot3d: R.from_matrix((rot3d.ndarray())).as_rotvec()),
    MapRule(ax_ang, lambda ax_ang, euler: R.from_rotvec(ax_ang.ndarray()).as_euler(euler.get('_coord')), euler,
                    lambda euler: R.from_euler(euler.ndarray(), euler.get('_coord')).as_rotvec(), with_target=True),
    MapRule(ax_ang, lambda ax_ang: as_ax_ang_4d(R.from_rotvec(ax_ang.ndarray())), ax_ang4d,
                    lambda ax_ang4d: from_ax_ang4d((ax_ang4d.ndarray())).as_rotvec()),
    # from or to rot 3d
    MapRule(rot3d, lambda rot3d, euler: R.from_matrix(rot3d.ndarray()).as_euler(euler.get('_coord')), euler,
                   lambda euler: R.from_euler(euler.ndarray(), euler.get('_coord')).as_rotvec(), with_target=True),
    MapRule(rot3d, lambda rot3d: as_ax_ang_4d(R.from_matrix(rot3d.ndarray())), ax_ang4d,
                   lambda ax_ang4d: from_ax_ang4d((ax_ang4d.ndarray())).as_matrix()),
    # ax_ang4d
    MapRule(ax_ang4d, lambda ax_ang4d, euler: from_ax_ang4d(ax_ang4d.ndarray()).as_euler(euler.get('_coord')), euler,
                      lambda euler: as_ax_ang_4d(R.from_euler(euler.ndarray(), euler.get('_coord'))), with_target=True),
]


class Quaternion(UnitType):
    default_unit = 'quat'
    default_dtype = DefDict(**quat)
    default_dim = 4
    # default_conversion_rules = conversion_rules


class Rotation3DMatix(UnitType):
    default_unit = 'rot3d'
    default_dtype = DefDict(**rot3d)
    default_value = np.eye(3) # identy matrix. Under the hood, it does DefDict.set(default_value)
    default_dim = (3, 3)
   # default_conversion_rules = conversion_rules

class Rotation2DMatix(UnitType):
    default_unit = 'rot2d'
    default_dtype = DefDict(**rot2d)
    default_value = np.eye(2) # identy matrix. Under the hood, it does DefDict.set(default_value)
    default_dim = (2, 2)

class Euler(UnitType):
    default_unit = 'euler'
    # _coord is a protected key (you don't see unless you implicitly access it)
    default_dtype = DefDict(**euler)
    default_value = [0, 0, 0, 'xyz']
    default_dim = 3
    default_conversion_rules = conversion_rules

class AxisAngle(UnitType):
    """
    This is a 3 element version of the Axis-Angle representation.
    The first three elements are unit vector times angle (hence dimension is angle)
    """
    default_unit = 'ax_ang'
    default_dtype = DefDict(**ax_ang)
    default_value = [0, 0, 0] # 0 rotation around z
    default_dim = 3
    default_conversion_rules = conversion_rules

class AxisAngle4D(UnitType):
    """
    This is a 4 element version of the Axis-Angle representation.
    The first three elements are unit vector and the 4th element is angle
    """
    default_unit = 'ax_ang4d'
    default_dtype = DefDict(**ax_ang4d)
    default_value = [0, 0, 1, 0]
    default_dim = 4
    default_conversion_rules = conversion_rules


if __name__ == '__main__':
    import sys
    #sys.setrecursionlimit(30)
    import time
    # st  = time.perf_counter()
    # p = DefDict(dict(x=Pos, quat=Quaternion))
    # print(time.perf_counter()-st)
    # p.set(dict(x=Pos(unit='cm', data=10)))
    # print(time.perf_counter()-st)
    # p.set(dict(quat=[1,0,0,0]))
    # print(time.perf_counter()-st)


    import unyt
    st = time.perf_counter()
    u = unyt.unyt_quantity.from_string('0.1m')
    print('unyt',time.perf_counter()-st)
    st = time.perf_counter()
    d = DefDict(dict(x=float,y=float,z=float))
    print(time.perf_counter()-st)
    # d.format(dict(x=0, y=0)).ndarray()
    # print(time.perf_counter()-st)
    st = time.perf_counter()
    r = DefDict(dict(x=Pos, rot3d=Rotation3DMatix))
    print(time.perf_counter()-st)
    st = time.perf_counter()
    q=Quaternion()
    print('unyt',time.perf_counter()-st)
    q.to(10)
    print('unyt',time.perf_counter()-st)
    pass
