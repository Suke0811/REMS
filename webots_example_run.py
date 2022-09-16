import logging
from sim import Simulation
from sim.inputs import KeyboardInput, FileCsvInput, FileInput
from sim.outputs import FileCsvOutput, AnimationOutput, FileOutput
from sim.utils import time_str
from sim.Config import SimConfig
from sim.robot_def.webots import CreateDef, Pioneer3AtDef, Pioneer3DxDef, EpuckDef, MooseDef, YoubotArmDef, YoubotBaseDef, YoubotDef
from sim.robot_def.WoodbotDef import WoodbotDef
from sim.device.webots.WebotsBinder import WebotsBinder
from sim.robots.differential_drive.DynabotHard import DynabotHard
from sim.robots.differential_drive.WoodbotHard import WoodbotHard
from sim.robots.differential_drive.CreateHard import CreateHard
from sim.device.state_estimator.ArucoDevice import ShareAruco
from sim.robots.ArucoBot import ArucoBot
from sim.robots import JacobianModel


logging.basicConfig(level=logging.INFO)

s = Simulation(debug_mode=False)    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
#i = FileInput('out/model09_14_2022_23_29_17.csv', loop=False)
i = KeyboardInput(False)
#i = JoystickInput()

s.set_input(i)  # specify inputs to run
# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model

out_dir = 'out/'
webots_csv = FileOutput(out_dir+'webots'+time_str()+'.csv')      # save to test.csv at the same dir as the

# add robots to simulation
#robot_ref = s.add_robot(ScalerManipulator, (ScalerHard, '/dev/MOTOR_0', 2), arm2_csv)


# s.add_robot(MooseDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
# s.add_robot(CreateDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
# s.add_robot(Pioneer3DxDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
# s.add_robot(Pioneer3AtDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
# s.add_robot(EpuckDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
# #s.add_robot(WoodbotDef, JacobianModel, (AnimationOutput('video/test'+time_str()+'.gif'), FileOutput('out/model'+time_str()+'.csv')))
# s.add_robot(WoodbotDef, WebotsBinder, (AnimationOutput('video/test'+time_str()+'.gif'),  FileOutput('out/sim'+time_str()+'.csv')))
# #s.add_robot(WoodbotDef, WoodbotHard, AnimationOutput('video/test'+time_str()+'.gif'))
#
# #s.add_robot(CreateDef, JacobianModel, AnimationOutput('video/test'+time_str()+'.gif'))
#
# s.add_robot(YoubotArmDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
# s.add_robot(YoubotBaseDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
# s.add_robot(YoubotDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))

#
# s.add_robot(MooseDef, WebotsBinder)
# s.add_robot(CreateDef, WebotsBinder)
# s.add_robot(Pioneer3AtDef, WebotsBinder)
# s.add_robot(Pioneer3DxDef, WebotsBinder)
# s.add_robot(EpuckDef, WebotsBinder)
#frf s.add_robot(WoodbotDef, WebotsBinder)
#
s.add_robot(YoubotDef, WebotsBinder,  FileOutput('out/1youbot.json'), inpt=FileInput('out/0youbot.json'))
# s.add_robot(YoubotArmDef, WebotsBinder)
# s.add_robot(YoubotDef, WebotsBinder)

#aruco = s.add_robot(None, (ArucoBot, [3, 2, 1], 6))

#
# r = s.add_robot(WoodbotDef, WoodbotHard)
# r.add_device(ShareAruco(observe_state=aruco.observe_state, track_id=1))
# r = s.add_robot(CreateDef, (CreateHard, '/dev/ttyUSB0'))
# r.add_device(ShareAruco(observe_state=aruco.observe_state, track_id=3))
# r = s.add_robot(Pioneer3AtDef, (DynabotHard, '/dev/ttyUSB1'), FileOutput('out/07.csv'))
# r.add_device(ShareAruco(observe_state=aruco.observe_state, track_id=2))



# r = s.add_robot(WoodbotDef, WoodbotHard, (AnimationOutput('video/test'+time_str()+'.gif'),  FileOutput('out/woodbot'+time_str()+'.csv')))
# r.add_device(ShareAruco(observe_state=aruco.obseeddddeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeddddedddddrve_state, track_id=1))



# r = s.add_robot(EpuckDef, (CreateHard, 'COM7', 0),  AnimationOutput('video/test'+time_str()+'.gif'))
# r.add_device(ShareAruco(observe_state=aruco.observe_state, track_id=3))

# r = s.add_robot(CreateDef, (DynabotHard, 'COM3'),  AnimationOutput('video/test'+time_str()+'.gif'))
#
#


#s.add_robot(WoodbotDef, WoodbotHard)
##s.add_robot(EpuckDef, WebotsBinder)

#s.add_robot(Pioneer3DxDef, WebotsBinder)
#s.add_robot(Pioneer3AtDef, WebotsBinder)

s.run(SimConfig(max_duration=10, dt=0.01, realtime=True, start_time=0, run_speed=1))  # run 10sec, at the end of run, automatically do outputs.
