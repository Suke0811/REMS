from sim.type import DefDict
"""Where to store standard definitions"""

# Defined type
class dimentional_float(float):
    unit = None
class velocity(float): pass
class position(float): pass
class acceleration(float): pass
class torque(float): pass
class angular_velocity(float): pass
class angular_acceleration(float): pass
class angular_torque(float): pass
class rotation(float): pass
class euler(rotation): pass
class quaternion(rotation): pass



TIMESTAMP = dict(timestamp=float)

POS_2D = dict(x=position, y=position)
ROT_2D = dict(c=rotation)
POS_3D = dict(x=position, y=position, z=position)
EULER_3D = dict(a=rotation, b=rotation, c=rotation)
QUAT = dict(qx=float, qy=float, qz=float, w=float)
ROT_MAT_2D = dict(r11=float, r12=float,
                  r21=float, r22=float)
ROT_MAT_3D = dict(r11=float, r12=float, r13=float,
                  r21=float, r22=float, r23=float,
                  r31=float, r32=float, r33=float,)
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



def define(prefix, num, type_=float):
    ret = {}
    for i in range(num):
        key = prefix + str(i)
        ret[key] = type_
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
