import logging, os, time
import pandas as pd
from sim.Simulation import Sim
from sim.inputs import FileInput
from sim.outputs import FileOutput
from sim.modules import *
from sim.controllers import AutoTuning
import matplotlib.pyplot as plt

PRINT = True

LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()

if PRINT:
    logging.basicConfig(level=LOGLEVEL)

s = Sim()    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
i = FileInput('target_robot.csv',loop=True)
#i = KeyboardInput()
#i = JoystickInput()

s.set_input(i)  # specify inputs to run

# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model
target_robot = KinematicsModelWithParams()
ref_robot = WebotsModel()

target_csv = FileOutput('target_robot.csv')       # save to test.csv at the same dir as the main
ref_csv = FileOutput('ref_robot.csv')       # save to test.csv at the same dir as the main

#target_anim = AnimationOutput()             # to animate kinematics model
target_robot.to_thread = False


# add robots to simulation
s.add_robot(ref_robot, (ref_csv,))
s.add_robot(target_robot, (target_csv, ))

# define autotuning process
at_process = AutoTuning(target_robot, ref_robot)

# add process to the simulation
s.add_process(at_process)

t_start=time.time()
s.run(max_duration=150, realtime=True)  # run 10sec, at the end of run, automatically do outputs.

print(time.time()-t_start)

data_target = pd.read_csv('target_robot.csv')
data_ref = pd.read_csv('ref_robot.csv')
x_ref = data_ref['x'].to_numpy()
y_ref = data_ref['y'].to_numpy()
th_ref = data_ref['theta'].to_numpy()
dth_ref = data_ref['dtheta'].to_numpy()
x_tar = data_target['x'].to_numpy()
y_tar = data_target['y'].to_numpy()
th_tar = data_target['theta'].to_numpy()
dth_tar = data_target['dtheta'].to_numpy()
h2_norm = data_target['h2_norm'].to_numpy()
h2_norm_x = data_target['h2_norm_x'].to_numpy()
h2_norm_y = data_target['h2_norm_y'].to_numpy()
h2_norm_dth = data_target['h2_norm_dth'].to_numpy()
time = data_target['timestamp'].to_numpy()

plt.figure(1)
plt.plot(time,x_ref)
plt.plot(time,x_tar)
plt.xlabel('time, [s]')
plt.ylabel('x, [m]')
plt.legend(['x ref', 'x target'])
plt.title('x')
plt.figure(2)
plt.plot(time,y_ref)
plt.plot(time,y_tar)
plt.xlabel('time, [s]')
plt.ylabel('y, [m]')
plt.legend(['y ref', 'y target'])
plt.title('y')
plt.figure(3)
plt.plot(time,dth_ref)
plt.plot(time,dth_tar)
plt.xlabel('time, [s]')
plt.ylabel('dth, [rad]')
plt.legend(['dth ref', 'dth target'])
plt.title('dth')
plt.figure(4)
plt.plot(time,th_ref)
plt.plot(time,th_tar)
plt.xlabel('time, [s]')
plt.ylabel('th, [rad]')
plt.legend(['th ref', 'th target'])
plt.title('th')
plt.figure(5)
plt.plot(time,h2_norm)
plt.xlabel('time, [s]')
plt.ylabel('h2 norm')
plt.title('h2 norm')
plt.figure(6)
plt.plot(time,h2_norm_x)
plt.xlabel('time, [s]')
plt.ylabel('h2 norm x')
plt.title('h2 norm x')
plt.figure(7)
plt.plot(time,h2_norm_y)
plt.xlabel('time, [s]')
plt.ylabel('h2 norm y')
plt.title('h2 norm y')
plt.figure(8)
plt.plot(time,h2_norm_dth)
plt.xlabel('time, [s]')
plt.ylabel('h2 norm dth')
plt.title('h2 norm dth')
plt.show()
