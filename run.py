import logging, os
from sim.SimRay import Sim
from sim.inputs import FileInput
from sim.outputs import FileOutput
from sim.robots.bind_robot import bind_robot
from sim.robots.scalear_leg.ScalerManipulatorDef import ScalerManipulator
from sim.tuning.AutoTuning import AutoTuning
from sim.robots.scalear_leg.ScalarHard import ScalerHard
from sim.bind.kinematic_model.KinematicModel import KinematicModel
from sim.robots.scalear_leg.Pybullet import Pybullet
from sim.utils import time_str
from sim.Config import SimConfig

PRINT = True

LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()

if PRINT:
    logging.basicConfig(level=LOGLEVEL)

s = Sim()    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
#i = FileInput('sim/utils/target_robot_circle_line.csv', loop=True)

#i_video = FileInput('trajectory/r2k/target_robot_video.csv', loop=False)
i_helix = FileInput('trajectory/target_robot_circle_helix.csv', loop=True)
#i = FileInput('trajectory/r2k/ref_robot.csv', loop=False)
#i = KeyboardInput()
#i = JoystickInput()



s.set_input(i_helix)  # specify inputs to run

# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model

#ref_robot = bind_robot(ScalerManipulator, ScalerHard, '/dev/ttyUSB1', 3)

arm_2 = bind_robot(ScalerManipulator, ScalerHard, '/dev/ttyUSB0', 2)
target_robot = bind_robot(ScalerManipulator, KinematicModel)
#at_process = AutoTuning(target_robot, ref_robot, real_to_sim=False)
pybullet_robots = bind_robot(ScalerManipulator, Pybullet)
pybullet_robots_2 = bind_robot(ScalerManipulator, Pybullet)

out_dir = 'out/'
target_csv = FileOutput(out_dir+'target_'+time_str()+'.csv')      # save to test.csv at the same dir as the
ref_csv = FileOutput(out_dir+'ref_'+time_str()+'.csv')
arm2_csv = FileOutput(out_dir+'arm2_'+time_str()+'.csv')


# add robots to simulation


# s.add_robot(ref_robot, (ref_csv,))
s.add_robot(arm_2, (arm2_csv,))
#s.add_robot(target_robot, (target_csv,))
#s.add_robot(pybullet_robots,)
#s.add_robot(pybullet_robots_2,)

N = 0
for n in range(N):
    s.add_robot(bind_robot(ScalerManipulator, Pybullet))

# add processalse
#s.add_process(at_process)

s.run(SimConfig(max_duration=100, dt=0.02, realtime=True, start_time=0, run_speed=1))  # run 10sec, at the end of run, automatically do outputs.


#AutotunePlot(ref_csv.filepath, target_csv.filepath)
