import logging
from sim import Simulation
from sim.inputs import KeyboardInput, FileCsvInput
from sim.outputs import FileCsvOutput, AnimationOutput
from sim.utils import time_str
from sim.Config import SimConfig
from sim.robot_def.webots import CreateDef, Pioneer3AtDef, Pioneer3DxDef, EpuckDef
from sim.device.webots.WebotsBinder import WebotsBinder
from sim.robots.differential_drive.DynamixelbotHard import DynamixelbotHard
from sim.robots.differential_drive.WoodbotHard import WoodbotHard
from sim.robots.differential_drive.CreateHard import CreateHard

import ray


logging.basicConfig(level=logging.INFO)

ray.init(local_mode=True, num_gpus=1)
s = Simulation()    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
i = FileCsvInput('out/sim08_29_2022_20_53_41.csv', loop=False)
#i = KeyboardInput()
#i = JoystickInput()

s.set_input(i)  # specify inputs to run
# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model

out_dir = 'out/'
webots_csv = FileCsvOutput(out_dir + 'webots' + time_str() + '.csv')      # save to test.csv at the same dir as the

# add robots to simulation
#robot_ref = s.add_robot(ScalerManipulator, (ScalerHard, '/dev/MOTOR_0', 2), arm2_csv)
s.add_robot(CreateDef, WebotsBinder, (AnimationOutput('video/test'+time_str()+'.gif'), FileCsvOutput('out/sim'+time_str()+'.csv')) )
#s.add_robot(CreateDef, (CreateHard, '/dev/ttyUSB0', 6), (AnimationOutput('video/test'+time_str()+'.gif'), FileCsvOutput('out/hard'+time_str()+'.csv')))
#s.add_robot(EpuckDef, WebotsBinder)

#s.add_robot(Pioneer3DxDef, WebotsBinder)
#s.add_robot(Pioneer3AtDef, WebotsBinder)

s.run(SimConfig(max_duration=10, dt=0.01, realtime=True, start_time=0, run_speed=1))  # run 10sec, at the end of run, automatically do outputs.

