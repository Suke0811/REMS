from sim.type import DefDict
"""Where to store standard definitions"""

# Defined type
class velocity(float): pass
class position(float): pass
class acceleration(float): pass
class torque(float): pass



TIMESTAMP = dict(timestamp=float)

STATE_2D = dict(x=float, y=float, th=float)
STATE_3D_EULER = dict(x=float, y=float, z=float, a=float, b=float, c=float)
STATE_3D_QUAT = dict(x=float, y=float, z=float, qx=float, qy=float, qz=float, w=float)

STATE_VEL_2D = dict(d_x=float, d_y=float, d_th=float)
STATE_VEL_3D_EULER = dict(d_x=float, d_y=float, d_z=float, d_a=float, d_b=float, d_c=float)
STATE_VEL_3D_QUAT = dict(d_x=float, d_y=float, d_z=float, d_qx=float, d_qy=float, d_qz=float, d_w=float)

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
