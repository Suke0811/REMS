import numpy as np
import matplotlib.pyplot as plt
import pickle
from sim.robots.scalear_leg.kinematics.SCALAR_kinematics import ScalerKinematics
from sim.typing import DefDict
from sim.typing.definitions import *
import pandas as pd
# add straight line trajectory
add_line = True
scalerKin = ScalerKinematics()
# specify radius of the circle (meters)
r = 0.10
offset = 50/1000
# scalar starting position (mm)
start_pos = [r+offset,0.0,-0.250]
# specify leg
which_leg = 3
# specify dt in seconds
dt = 0.01
# specify angular velocity (rad/s)
dth = 0.8
# specify linear velocity for line one way
dx = 0.1
# specify total distance for line (m) back and forth
x_dist = 0.10

prev_joints = DefDict(joint_pos(6))

T_shi_wrist3 = T_mat_rule(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,start_pos[0],start_pos[1],start_pos[2])
joint_angles = scalerKin.scalar_inverse_kinematics(which_leg,T_shi_wrist3,is_first_ik=True,prev_angles=None)
prev_joints.set(joint_angles)


# calculate total time to make 1 round
T = (1.0/dth)*2*np.pi
t = np.linspace(0,2*np.pi,int(T/dt))

x_list = []
y_list = []

for i in range(len(t)):
    x_list.append(r*np.cos(t[i])+start_pos[0])
    y_list.append(r*np.sin(t[i])+start_pos[1])

dx_list = []
dy_list = []

for i in range(len(t)-1):
    dx_list.append((x_list[i+1]-x_list[i])/dt)
    dy_list.append((y_list[i+1]-y_list[i])/dt)

ref_state = []

for i in range(len(t)-1):
    ref_state.append([x_list[i],y_list[i],start_pos[2],dx_list[i],dy_list[i],0.0])

# build list of joint angles
j0,j1,j2,j3,j4,j5,dj0,dj1,dj2,dj3,dj4,dj5 = [],[],[],[],[],[],[],[],[],[],[],[]
j0.append(prev_joints.list()[0])
j1.append(prev_joints.list()[1])
j2.append(prev_joints.list()[2])
j3.append(prev_joints.list()[3])
j4.append(prev_joints.list()[4])
j5.append(prev_joints.list()[5])
dj0.append(0)
dj1.append(0)
dj2.append(0)
dj3.append(0)
dj4.append(0)
dj5.append(0)
joint_list = [0]*24

for i in range(len(t)-2):
    cur_pos = [ref_state[i+1][0],ref_state[i+1][1],ref_state[i+1][2]]
    T_shi_wrist3 = T_mat_rule(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, cur_pos[0], cur_pos[1], cur_pos[2])
    joint_list[which_leg*6:which_leg*6+6] = prev_joints.list()
    joint_angles = scalerKin.scalar_inverse_kinematics(which_leg, T_shi_wrist3, prev_angles=prev_joints.list())
    prev_joints.set(joint_angles)
    j0.append(joint_angles[0])
    j1.append(joint_angles[1])
    j2.append(joint_angles[2])
    j3.append(joint_angles[3])
    j4.append(joint_angles[4])
    j5.append(joint_angles[5])
    dj0.append(0.0)
    dj1.append(0.0)
    dj2.append(0.0)
    dj3.append(0.0)
    dj4.append(0.0)
    dj5.append(0.0)

# timestamps
timestamp = []
cur_time = 0.0
ref_x, ref_y, ref_z =  [],[],[]
for i in range(len(t)-1):
    ref_x.append(ref_state[i][0])
    ref_y.append(ref_state[i][1])
    ref_z.append(ref_state[i][2])
    cur_time += dt
    timestamp.append(cur_time)


if add_line:
    x_list = []
    y_list = []
    # calculate total time to make 2 lines (one forward, one backward)
    T = (1.0 / dx) * x_dist
    x_list1 = np.linspace(start_pos[0], start_pos[0]+x_dist, int(T/dt))
    y_list1 = np.linspace(start_pos[1], start_pos[1], int(T/dt))
    x_list1 = x_list1.tolist()
    y_list1 = y_list1.tolist()
    x_list2 = np.linspace(start_pos[0] + x_dist, start_pos[0], int(T / dt))
    y_list2 = np.linspace(start_pos[1], start_pos[1], int(T/dt))
    x_list2 = x_list2.tolist()
    y_list2 = y_list2.tolist()

    x_list.extend(x_list1)
    x_list.extend(x_list2)
    y_list.extend(y_list1)
    y_list.extend(y_list2)

    dx_list = []
    dy_list = []
    for i in range(len(x_list)-1):
        dx_list.append((x_list[i+1]-x_list[i])/dt)
        dy_list.append((y_list[i+1]-y_list[i])/dt)

    ref_state = []
    for i in range(len(x_list)-1):
        ref_state.append([x_list[i], y_list[i], start_pos[2], dx_list[i], dy_list[i], 0.0])

    # build list of joint angles
    j02, j12, j22, j32, j42, j52, dj02, dj12, dj22, dj32, dj42, dj52 = [], [], [], [], [], [], [], [], [], [], [], []
    j02.append(prev_joints.list()[0])
    j12.append(prev_joints.list()[1])
    j22.append(prev_joints.list()[2])
    j32.append(prev_joints.list()[3])
    j42.append(prev_joints.list()[4])
    j52.append(prev_joints.list()[5])
    dj02.append(0)
    dj12.append(0)
    dj22.append(0)
    dj32.append(0)
    dj42.append(0)
    dj52.append(0)
    joint_list = [0] * 24

    for i in range(len(x_list) - 2):
        cur_pos = [ref_state[i + 1][0], ref_state[i + 1][1], ref_state[i + 1][2]]
        T_shi_wrist3 = T_mat_rule(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, cur_pos[0], cur_pos[1], cur_pos[2])
        joint_list[which_leg * 6:which_leg * 6 + 6] = prev_joints.list()
        joint_angles = scalerKin.scalar_inverse_kinematics(which_leg, T_shi_wrist3,
                                                           prev_angles=prev_joints.list())
        prev_joints.set(joint_angles)
        j02.append(joint_angles[0])
        j12.append(joint_angles[1])
        j22.append(joint_angles[2])
        j32.append(joint_angles[3])
        j42.append(joint_angles[4])
        j52.append(joint_angles[5])
        dj02.append(0.0)
        dj12.append(0.0)
        dj22.append(0.0)
        dj32.append(0.0)
        dj42.append(0.0)
        dj52.append(0.0)

    # timestamps
    timestamp2 = []
    cur_time = 0.0
    ref_x2, ref_y2, ref_z2 = [], [], []
    for i in range(len(x_list)-1):
        ref_x2.append(ref_state[i][0])
        ref_y2.append(ref_state[i][1])
        ref_z2.append(ref_state[i][2])
        cur_time += dt
        timestamp2.append(cur_time)


dict = {'timestamp':timestamp, 'x':ref_x, 'y':ref_y,
                'z':ref_z, 'j.0':j0, 'j.1':j1,
                'j.2':j2, 'j.3':j3,'j.4':j4,'j.5':j5,'d_j.0':dj0,'d_j.1':dj1,'d_j.2':dj2,
                'd_j.3': dj3, 'd_j.4': dj4, 'd_j.5': dj5}
df = pd.DataFrame(dict)
df.to_csv('target_robot_circle.csv',index=False)

if add_line:
    dict2 = {'timestamp':timestamp2, 'x':ref_x2, 'y':ref_y2,
                'z':ref_z2, 'j.0':j02, 'j.1':j12,
                'j.2':j22, 'j.3':j32,'j.4':j42,'j.5':j52,'d_j.0':dj02,'d_j.1':dj12,'d_j.2':dj22,
                'd_j.3': dj32, 'd_j.4': dj42, 'd_j.5': dj52}
    df = pd.DataFrame(dict2)
    df.to_csv('target_robot_line.csv',index=False)

    j0.extend(j02)
    j1.extend(j12)
    j2.extend(j22)
    j3.extend(j32)
    j4.extend(j42)
    j5.extend(j52)
    dj0.extend(dj02)
    dj1.extend(dj12)
    dj2.extend(dj22)
    dj3.extend(dj32)
    dj4.extend(dj42)
    dj5.extend(dj52)
    ref_x.extend(ref_x2)
    ref_y.extend(ref_y2)
    ref_z.extend(ref_z2)

    timestamp3 = []
    cur_time = 0.0
    for i in range(len(j0)):
        cur_time += dt
        timestamp3.append(cur_time)

    dict3 = {'timestamp': timestamp3, 'x': ref_x, 'y': ref_y,
             'z': ref_z, 'j.0': j0, 'j.1': j1,
             'j.2': j2, 'j.3': j3, 'j.4': j4, 'j.5': j5, 'd_j.0': dj0, 'd_j.1': dj1, 'd_j.2': dj2,
             'd_j.3': dj3, 'd_j.4': dj4, 'd_j.5': dj5}
    df = pd.DataFrame(dict3)
    df.to_csv('target_robot_circle_line.csv', index=False)



#plt.plot(ref_x,ref_y)
#plt.show()
