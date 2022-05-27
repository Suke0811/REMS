import numpy as np
import matplotlib.pyplot as plt
import pickle
from sim.robots.scalear_leg.kinematics.SCALAR_kinematics import ScalerKinematics
from sim.type.DefDict import DefDict
from sim.type.definitions import *
import pandas as pd

scalerKin = ScalerKinematics()
# specify radius of the circle (meters)
r = 0.10
offset = 50/1e3
# scalar starting position (mm)
start_pos = [r+offset,0.0,-0.250]
# specify leg
which_leg = 3
# specify dt in seconds
dt = 0.01
# specify angular velocity (rad/s)
dth = 0.8

#cur_joints = DefDict(joint_pos(6), dict(a=float, b=float, c=float, d=float, e=float, f=float))
prev_joints = DefDict(joint_pos(6))

T_shi_wrist3 = T_mat_rule(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,start_pos[0],start_pos[1],start_pos[2])
joint_angles = scalerKin.scalar_inverse_kinematics(which_leg,T_shi_wrist3,is_first_ik=True,prev_angles=None)
prev_joints.data = joint_angles

#cur_joints.data = {'j.0': 1.0, 'j.1': 0.0, 'j.2': 0.0, 'j.3': 0.0, 'j.4': 0.0, 'j.5': 0.0}

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
j0.append(prev_joints.data.as_list()[0])
j1.append(prev_joints.data.as_list()[1])
j2.append(prev_joints.data.as_list()[2])
j3.append(prev_joints.data.as_list()[3])
j4.append(prev_joints.data.as_list()[4])
j5.append(prev_joints.data.as_list()[5])
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
    joint_list[which_leg*6:which_leg*6+6] = prev_joints.data.as_list()
    joint_angles = scalerKin.scalar_inverse_kinematics(which_leg, T_shi_wrist3,  prev_angles=prev_joints.data.as_list())
    prev_joints.data = joint_angles
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


dict = {'timestamp':timestamp, 'x':ref_x, 'y':ref_y,
                'z':ref_z, 'j.0':j0, 'j.1':j1,
                'j.2':j2, 'j.3':j3,'j.4':j4,'j.5':j5,'d_j.0':dj0,'d_j.1':dj1,'d_j.2':dj2,
                'd_j.3': dj3, 'd_j.4': dj4, 'd_j.5': dj5}
df = pd.DataFrame(dict)
df.to_csv('target_robot_circle.csv',index=False)


plt.plot(ref_x,ref_y)
plt.show()
