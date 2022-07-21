import copy

from sim.typing import DefDict, BindRule
from typing import Any
import numpy as np

DEFINITION = 'definition'
DTYPE = 'dtype'
NAME = 'name'
PREFIXES = 'prefixes'
SUFFIXES = 'suffixes'
FORMAT_RULE = 'format_rule'

class QUAT:
    qx = "qx"; qy = 'qy'; qz = 'qz'; qw = 'qw'
    KEY = (qx,qy,qz,qw)

class EULER:
    a = 'a'; b = 'b'; c = 'c'
    KEY = (a,b,c)


class bindable(float):
    bind_from = None
    bind_to = []

class velocity(float): pass
class position(float): pass
class acceleration(float): pass
class torque(float): pass
class angular_velocity(float): pass

class angular_acceleration(float): pass
class angular_torque(float): pass

class angular_position(bindable): pass
class rotation_matrix(angular_position): pass
class euler(angular_position): pass
class quaternion(angular_position):
    bind_from = QUAT.KEY
    bind_to = [EULER]


TIMESTAMP = 'timestamp'



POS_2D = dict(x=float, y=float)
ROT_2D = dict(c=float)
POS_3D = dict(x=float, y=float, z=float)


QUAT = dict(qx=float, qy=float, qz=float, qw=float)
EULER_3D = dict(a=float, b=float, c=float)

ROT_MAT_2D = dict(r11=float, r12=float,
                  r21=float, r22=float)
ROT_MAT_3D = dict(r11=float, r12=float, r13=float,
                  r21=float, r22=float, r23=float,
                  r31=float, r32=float, r33=float,)

def T_mat_rule(r11, r12, r13,
               r21, r22, r23,
               r31, r32, r33,
               x, y, z):
    return np.array([[r11, r12, r13, x],
               [r21, r22, r23, y],
               [r31, r32, r33, z],
               [0,0,0,1]])

def Tmat2dict(row0,row1,row2,row3):
    return dict(r11=row0[0], r12=row0[1], r13=row0[2],
                        r21=row1[0], r22=row1[1], r23=row1[2],
                        r31=row2[0], r32=row2[1], r33=row2[2],
                        x=row0[3], y=row1[3], z=row2[3])


T_keys = list(ROT_MAT_3D.keys()) + list(POS_3D.keys())
T_MAT = DefDict(definition=(ROT_MAT_3D,POS_3D), format_rule=BindRule(T_keys, bind_func=T_mat_rule, inv_bind_func=Tmat2dict))



MOTOR = dict(pos=float, vel=float, enable=bool)
JOINTS = {'j.0': DefDict(MOTOR), 'j.1': DefDict(MOTOR), 'j.2': DefDict(MOTOR)}
ROT_VECTOR = {'r.vec_0':float, 'r.vec_1':float, 'r.vec_2':float}
ROT_UNIT_VECTOR = {'r.uvec_0':float, 'r.uvec_1':float, 'r.uvec_2':float, 'r.uvec_th':float}

VEL_POS_2D = dict(d_x=float, d_y=float)
VEL_ROT_2D = dict(d_th=float)
VEL_POS_3D = dict(d_x=velocity, d_y=velocity, d_z=velocity)
VEL_ROT_3D = dict(d_a=float, d_b=float, d_c=float)



JACOB_2D = dict(J11=float, J12=float, J13=float,
                J21=float, J22=float, J23=float,
                J31=float, J32=float, J33=float)
JACOB_3D = dict(J11=float, J12=float, J13=float, J14=float, J15=float, J16=float,
                J21=float, J22=float, J23=float, J24=float, J25=float, J26=float,
                J31=float, J32=float, J33=float, J34=float, J35=float, J36=float,
                J41=float, J42=float, J43=float, J44=float, J45=float, J46=float,
                J51=float, J52=float, J53=float, J54=float, J55=float, J56=float,
                J61=float, J62=float, J63=float, J64=float, J65=float, J66=float)


def define(prefix, num, dtype=Any, separater='.'):
    d = {}
    if isinstance(num, list):
        for n in num:
            key = prefix + separater + str(n)
            d[key] = copy.deepcopy(dtype)
    else:
        for i in range(num):
            key = prefix + separater + str(i)
            d[key] = copy.deepcopy(dtype)
    #ret = {DEFINITION: d, DTYPE: dtype, PREFIXES: [prefix]}
    ret = d
    return ret

def joint_pos(num):
    return define('j', num, position)

def joint_vel(num):
    return define('d_j', num, velocity)

def joint_acc(num):
    return define('dd_j', num, acceleration)

def joint_torque(num):
    return define('j_t', num, torque)

a = joint_pos(6)
pass
