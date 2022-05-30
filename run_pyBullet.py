import logging, os, time
import pandas as pd
from sim.Sim import Sim
from sim.inputs import FileInput, KeyboardInput
from sim.outputs import FileOutput
import matplotlib.pyplot as plt
from sim.utils.tictoc import tictoc
from sim.robots.bind_robot import bind_robot
from sim.robots.scalear_leg.ScalerManipulatorDef import ScalerManipulator
from sim.tuning.AutoTuning import AutoTuning
from sim.robots.scalear_leg.ScalarHard import ScalerHard
from sim.robots.scalear_leg.Pybullet import Pybullet
from sim.robots.scalear_leg.Pybullet_AutoTune import Pybullet2
from sim.robots.bind.kinematic_model.KinematicModel import KinematicModel
import numpy as np

PRINT = True
real_to_sim = True

LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()

if PRINT:
    logging.basicConfig(level=LOGLEVEL)

s = Sim(DT=0.25)    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
i = FileInput('sim/utils/target_robot_circle.csv', loop=True)
#i = KeyboardInput()
#i = JoystickInput()

s.set_input(i)  # specify inputs to run

# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model

if real_to_sim:
    target_robot = bind_robot(ScalerManipulator, Pybullet2)
    ref_robot = bind_robot(ScalerManipulator, ScalerHard, '/dev/ttyUSB0')

else:
    ref_robot = bind_robot(ScalerManipulator, Pybullet)
    target_robot = bind_robot(ScalerManipulator, KinematicModel)

at_process = AutoTuning(target_robot, ref_robot, real_to_sim)


target_csv = FileOutput('test_robot.csv')       # save to test.csv at the same dir as the
ref_csv = FileOutput('ref_robot.csv')

# add robots to simulation
s.add_robot(ref_robot, (ref_csv,))
s.add_robot(target_robot, (target_csv,))

# add process
s.add_process(at_process)

s.run(max_duration=100, realtime=True)  # run 10sec, at the end of run, automatically do outputs.


data_target = pd.read_csv('test_robot.csv')
data_ref = pd.read_csv('ref_robot.csv')

x_ref = data_ref['x'].to_numpy()
y_ref = data_ref['y'].to_numpy()

dx_ref = data_ref['d_x'].to_numpy()
dy_ref = data_ref['d_y'].to_numpy()

dx_tar = data_target['d_x'].to_numpy()
dy_tar = data_target['d_y'].to_numpy()

x_tar = data_target['x'].to_numpy()
y_tar = data_target['y'].to_numpy()

h2_norm = data_target['h2_norm'].to_numpy()
h2_norm_dx = data_target['h2_norm_x'].to_numpy()
h2_norm_dy = data_target['h2_norm_y'].to_numpy()
time_stamp = data_target['timestamp'].to_numpy()

plt.figure(1)
plt.plot(time_stamp,h2_norm_dx)
plt.xlabel('time, [s]')
plt.ylabel('h2 norm dx')
plt.title('h2 norm dx')
plt.figure(2)
plt.plot(time_stamp,h2_norm_dy)
plt.xlabel('time, [s]')
plt.ylabel('h2 norm dy')
plt.title('h2 norm dy')
plt.figure(3)
plt.plot(time_stamp,h2_norm)
plt.xlabel('time, [s]')
plt.ylabel('h2 norm')
plt.title('h2 norm')
plt.figure(4)
plt.plot(time_stamp,dx_ref)
plt.plot(time_stamp,dx_tar)
plt.xlabel('time, [s]')
plt.ylabel('dx, [m]')
plt.legend(['dx ref', 'dx target'])
plt.title('dx')
plt.figure(5)
plt.plot(time_stamp,dy_ref)
plt.plot(time_stamp,dy_tar)
plt.xlabel('time, [s]')
plt.ylabel('dy, [m]')
plt.legend(['dy ref', 'dy target'])
plt.figure()
plt.plot(time_stamp,y_ref)
plt.plot(time_stamp,y_tar)
plt.xlabel('time, [s]')
plt.ylabel('y, [m]')
plt.legend(['y ref', 'y target'])
plt.show()

if not real_to_sim:
    with open('sim/NN_param.npy', 'wb') as f:
        print('NN Model Saved')
        np.save(f, target_robot.PARAMS)

if real_to_sim:
    with open('sim/NN2_param.npy', 'wb') as f:
        print('NN2 Model Saved')
        np.save(f, target_robot.PARAMS)