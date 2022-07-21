import logging, os
from sim import Simulation
from sim.inputs import FileInput
from sim.inputs.JoyManipulator import JoyManipulator
from sim.outputs import FileOutput
from sim.robots.scalear_leg.ScalerManipulatorDef import ScalerManipulator
from sim.robots.scalear_leg.ScalarHard import ScalerHard
from sim.bind.kinematic_model.KinematicModel import KinematicModel
from sim.robots.scalear_leg.Pybullet import Pybullet
from sim.utils import time_str
from sim.Config import SimConfig
import ray


logging.basicConfig(level=logging.INFO)

ray.init(local_mode=True)
s = Simulation()    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
#i = FileInput('sim/utils/target_robot_circle_line.csv', loop=True)

#i_video = FileInput('trajectory/r2k/target_robot_video.csv', loop=False)
i_helix = FileInput('trajectory/target_robot_circle_line.csv', loop=True)
#i = FileInput('trajectory/r2k/target_robot_video.csv', loop=False)
#i = KeyboardInput()
#i = JoystickInput()

s.set_input(i_helix)  # specify inputs to run

# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model

out_dir = 'out/'
target_csv = FileOutput(out_dir+'target_'+time_str()+'.csv')      # save to test.csv at the same dir as the
ref_csv = FileOutput(out_dir+'ref_'+time_str()+'.csv')
arm2_csv = FileOutput(out_dir+'arm2_'+time_str()+'.csv')


# add robots to simulation
#robot_ref = s.add_robot(ScalerManipulator, (ScalerHard, '/dev/MOTOR_0', 2), arm2_csv)

N = 1
for n in range(N):
    s.add_robot(ScalerManipulator, Pybullet)

s.run(SimConfig(max_duration=10, dt=0.02, realtime=True, start_time=0, run_speed=1))  # run 10sec, at the end of run, automatically do outputs.
