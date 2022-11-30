from StdUnit import *

import copy
from typing import Any


'''Separator'''
SEPARATOR = '.'
NPS = NON_PREFIX_SEPARATOR = '_'

'''Position'''
POS_2D = dict(x=Pos, y=Pos)
POS_2D_mm = dict(x=Pos(unit='mm'), y=Pos(unit='mm'))
POS_3D = dict(x=Pos, y=Pos, z=Pos)
POS_3D_mm = dict(x=Pos(unit='mm'), y=Pos(unit='mm'), z=Pos(unit='mm'))

'''Rotation'''
QUAT = dict(quat=dict(qw=0.0, qx=1, qy=0.0, qz=0.0))

ROT_MAT = dict(rot_mat=dict(r01=1.0, r02=0.0, r03=0.0,
                  r11=0.0, r12=1.0, r13=0.0,
                  r21=0.0, r22=0.0, r23=1.0,))

AXIS_ANGLE = dict(ax=0.0, ay=0.0, az=1.0, angle=Ang)
AXIS_ANGLE_VEC = dict(ax=0.0, ay=0.0, az=0.0)
EULER_3D = dict(a=float, b=float, c=float)

'''Position velocity'''

VEL_POS_2D = dict(d_x=Vel, d_y=Vel)
VEL_POS_3D = dict(d_x=Vel, d_y=Vel, d_z=Vel)

'''Angular velocity'''
VEL_ROT_2D = dict(d_th=AngVel)
VEL_ROT_3D = dict(d_a=AngVel, d_b=AngVel, d_c=AngVel)

'''Position Acceleration'''
ACC_POS_2D = dict(dd_x=Acc, dd_y=Acc)
ACC_POS_3D = dict(dd_x=Acc, dd_y=Acc, dd_z=Acc)

'''Angular Acceleration'''
ACC_ROT_2D = dict(dd_th=AngAcc)
ACC_ROT_3D = dict(dd_a=AngAcc, dd_b=AngAcc, dd_c=AngAcc)


'''jacobian'''
JACOB_2D = dict(Jb00=float, J02=float, J03=float,
                Jb11=float, J12=float, J13=float,
                Jb21=float, J22=float, J23=float)

JACOB_3D = dict(Jb01=float, J02=float, J03=float, J04=float, J05=float, J06=float,
                Jb11=float, J12=float, J13=float, J14=float, J15=float, J16=float,
                Jb21=float, J22=float, J23=float, J24=float, J25=float, J26=float,
                Jb31=float, J32=float, J33=float, J34=float, J35=float, J36=float,
                Jb41=float, J42=float, J43=float, J44=float, J45=float, J46=float,
                Jb51=float, J52=float, J53=float, J54=float, J55=float, J56=float)


def define(prefix, num, dtype=Any, separater='.', link=False):
    d = {}
    if isinstance(num, list):
        for n in num:
            key = prefix + separater + str(n)
            if link:
                d[key] = copy.copy(dtype)
            else:
                d[key] = copy.deepcopy(dtype)
    else:
        for i in range(num):
            key = prefix + separater + str(i)
            if link:
                d[key] = copy.copy(dtype)
            else:
                d[key] = copy.deepcopy(dtype)
    ret = d
    return ret
