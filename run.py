import logging, os
from rems import Simulation
from rems.inputs import FileCsvInput
from rems.inputs.JoyManipulator import JoyManipulator
from rems.outputs import FileCsvOutput
from rems.robots.scaler_leg.ScalerManipulatorDef import ScalerManipulator
from rems.robots.scaler_leg.ScalarHard import ScalerHard
from rems.device.kinematic_model.KinematicModel import KinematicModel
from rems.robots.scaler_leg.Pybullet import Pybullet
from rems.utils import time_str
from rems.Config import SimConfig

from rems.robots.scaler import ScalerMode, ScalerDef, SimScaler
import ray


logging.basicConfig(level=logging.INFO)

ray.init(local_mode=True)
s = Simulation()    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
#i = FileCsvInput('rems/utils/target_robot_circle_line.csv', loop=True)

#i_video = FileCsvInput('trajectory/r2k/target_robot_video.csv', loop=False)
i_helix = FileCsvInput('trajectory/target_robot_circle_line.csv', loop=True)
#i = FileCsvInput('trajectory/r2k/target_robot_video.csv', loop=False)
#i = KeyboardInput()
#i = JoystickInput()

s.set_input(i_helix)  # specify inputs to run

# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model

out_dir = 'out/'
target_csv = FileCsvOutput(out_dir + 'target_' + time_str() + '.csv')      # save to test.csv at the same dir as the
ref_csv = FileCsvOutput(out_dir + 'ref_' + time_str() + '.csv')
arm2_csv = FileCsvOutput(out_dir + 'arm2_' + time_str() + '.csv')


# add robots to simulation
#robot_ref = s.add_robot(ScalerManipulator, (ScalerHard, '/dev/MOTOR_0', 2), arm2_csv)

N = 1
for n in range(N):
    s.add_robot((ScalerDef, ScalerMode.Walking()), SimScaler)
    #s.add_robot(ScalerManipulator, Pybullet)

s.run(SimConfig(max_duration=10, dt=0.02, realtime=True, start_time=0, run_speed=1))  # run 10sec, at the end of run, automatically do outputs.
