import logging
from rems import Operator
from rems.inputs import KeyboardInput, FileCsvInput, FileInput, JoystickInput
from rems.outputs import FileCsvOutput, AnimationOutput, FileOutput
from rems.utils import time_str
from rems.Config import SimConfig
from rems.robot_def.WoodbotDef import WoodbotDef
from rems.device.webots.WebotsBinder import WebotsBinder


logging.basicConfig(level=logging.INFO)

s = Operator(debug_mode=True)    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
#i = FileInput('trajectory/sim09_14_2022_23_45_37.csv', loop=False)
#i = KeyboardInput(wait_for=True)
i = JoystickInput(0)

s.set_input(i)  # specify inputs to run
# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model

out_dir = 'out/'
webots_csv = FileOutput(out_dir+'webots'+time_str()+'.csv')      # save to test.csv at the same dir as the

# add robots to simulation
#robot_ref = s.add_robot(ScalerManipulator, (ScalerHard, '/dev/MOTOR_0', 2), arm2_csv)


# s.add_robot(MooseDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
#s.add_robot(CreateDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
# s.add_robot(Pioneer3DxDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
# s.add_robot(Pioneer3AtDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
# s.add_robot(EpuckDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
# #s.add_robot(WoodbotDef, JacobianModel, (AnimationOutput('video/test'+time_str()+'.gif'), FileOutput('out/model'+time_str()+'.csv')))
s.add_robot(WoodbotDef,)
# #s.add_robot(WoodbotDef, WoodbotHard, AnimationOutput('video/test'+time_str()+'.gif'))
#
# #s.add_robot(CreateDef, JacobianModel, AnimationOutput('video/test'+time_str()+'.gif'))
#
# s.add_robot(YoubotArmDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
# s.add_robot(YoubotBaseDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))
# s.add_robot(YoubotDef, WebotsBinder, AnimationOutput('video/test'+time_str()+'.gif'))


# #
# s.add_robot(MooseDef, WebotsBinder, inpt=FileInput('out/01.csv', init_state=False))
# s.add_robot(CreateDef, WebotsBinder, inpt=FileInput('out/02.csv', init_state=False))
# s.add_robot(Pioneer3AtDef, WebotsBinder, inpt=FileInput('out/03.csv', init_state=False))
# s.add_robot(Pioneer3DxDef, WebotsBinder, inpt=FileInput('out/04.csv', init_state=False))
# s.add_robot(EpuckDef, WebotsBinder, inpt=FileInput('out/05.csv', init_state=False))
# s.add_robot(WoodbotDef, WebotsBinder, inpt=FileInput('out/06.csv', init_state=False))
# #
# s.add_robot(YoubotBaseDef, WebotsBinder, inpt=FileInput('out/1youbot.yml', init_state=False))
# s.add_robot(YoubotArmDef, WebotsBinder, inpt=FileInput('out/1youbot.yml', init_state=False))
# s.add_robot(YoubotDef, WebotsBinder, inpt=FileInput('out/1youbot.yml', init_state=False))


#aruco = s.add_robot(None, (ArucoBot, [3, 2, 1], 0))
#
# r = s.add_robot(WoodbotDef, WoodbotHard, inpt=FileInput('out/0create+.csv'))
# r.add_device(ShareAruco(observe_state=aruco.observe_state, track_id=1))
# r = s.add_robot(CreateDef, (CreateHard, '/dev/ttyUSB0'), inpt=FileInput('out/0create+.csv'))#inpt=FileInput('out/01.csv'))
# r.add_device(ShareAruco(observe_state=aruco.observe_state, track_id=3))
#r = s.add_robot(Pioneer3AtDef, (DynabotHard, 'COM3'), (AnimationOutput('video/test'+time_str()+'.gif')))
#r.add_device(ShareAruco(observe_state=aruco.observe_state, track_id=2))



# FileOutput('out/07.csv')

# r = s.add_robot(WoodbotDef, WoodbotHard, (AnimationOutput('video/test'+time_str()+'.gif'),  FileOutput('out/woodbot'+time_str()+'.csv')))
# r.add_device(ShareAruco(observe_state=aruco.obseeddddeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeddddedddddrve_state, track_id=1))

#r = s.add_robot(EpuckDef, (CreateHard, 'COM7', 0),  AnimationOutput('video/test'+time_str()+'.gif'))
# r.add_device(ShareAruco(observe_state=aruco.observe_state, track_id=3))

# r = s.add_robot(CreateDef, (DynabotHard, 'COM3'),  AnimationOutput('video/test'+time_str()+'.gif'))
#
#


#s.add_robot(WoodbotDef, WoodbotHard)
##s.add_robot(EpuckDef, WebotsBinder)

#s.add_robot(Pioneer3DxDef, WebotsBinder)
#s.add_robot(Pioneer3AtDef, WebotsBinder)

s.run(SimConfig(max_duration=10, dt=0.01, realtime=False, start_time=0, run_speed=1))  # run 10sec, at the end of run, automatically do outputs.
