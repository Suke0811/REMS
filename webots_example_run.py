import logging
from sim import Simulation
from sim.inputs import KeyboardInput, FileCsvInput, FileInput
from sim.outputs import FileCsvOutput, AnimationOutput, FileOutput
from sim.utils import time_str
from sim.Config import SimConfig
from sim.robot_def.webots import CreateDef, Pioneer3AtDef, Pioneer3DxDef, EpuckDef
from sim.robot_def.WoodbotDef import WoodbotDef
from sim.device.webots.WebotsBinder import WebotsBinder
from sim.robots.differential_drive.DynabotHard import DynabotHard
from sim.robots.differential_drive.WoodbotHard import WoodbotHard
from sim.robots.differential_drive.CreateHard import CreateHard
from sim.device.state_estimator.ArucoDevice import ShareAruco
from sim.robots.ArucoBot import ArucoBot


logging.basicConfig(level=logging.INFO)

s = Simulation(debug_mode=False)    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
#i = FileInput('out/Pioneer 3-DX_08_10_2022_13_26_54.csv', loop=False)
i = KeyboardInput()
#i = JoystickInput()

s.set_input(i)  # specify inputs to run
# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model

out_dir = 'out/'
webots_csv = FileOutput(out_dir+'webots'+time_str()+'.csv')      # save to test.csv at the same dir as the

# add robots to simulation
#robot_ref = s.add_robot(ScalerManipulator, (ScalerHard, '/dev/MOTOR_0', 2), arm2_csv)

aruco = s.add_robot(None, (ArucoBot, [3, 2, 1]))

s.add_robot(CreateDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))

r = s.add_robot(CreateDef, (CreateHard, 'COM7', 0),  AnimationOutput('video/test'+time_str()+'.gif'))
r.add_device(ShareAruco(observe_state=aruco.observe_state, track_id=3))

r = s.add_robot(CreateDef, (DynabotHard, 'COM3'),  AnimationOutput('video/test'+time_str()+'.gif'))
r.add_device(ShareAruco(observe_state=aruco.observe_state, track_id=2))

r = s.add_robot(CreateDef, WoodbotHard,  AnimationOutput('video/test'+time_str()+'.gif'))
r.add_device(ShareAruco(observe_state=aruco.observe_state, track_id=1))

#s.add_robot(WoodbotDef, WoodbotHard)
##s.add_robot(EpuckDef, WebotsBinder)

#s.add_robot(Pioneer3DxDef, WebotsBinder)
#s.add_robot(Pioneer3AtDef, WebotsBinder)

s.run(SimConfig(max_duration=10, dt=0.01, realtime=True, start_time=0, run_speed=1))  # run 10sec, at the end of run, automatically do outputs.

