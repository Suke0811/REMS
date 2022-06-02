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


PRINT = True

LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()

if PRINT:
    logging.basicConfig(level=LOGLEVEL)

s = Sim(DT=0.03)    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
i = FileInput('sim/utils/target_robot_circle_line.csv', loop=True)

#i = FileInput('trajectory/target_robot_video.csv', loop=True)
#i = KeyboardInput()
#i = JoystickInput()



s.set_input(i)  # specify inputs to run

# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model

ref_robot = bind_robot(ScalerManipulator, ScalerHard, '/dev/ttyUSB0', 2)

#arm_2 = bind_robot(ScalerManipulator, ScalerHard, '/dev/ttyUSB1', 3)
target_robot = bind_robot(ScalerManipulator, KinematicModel)
at_process = AutoTuning(target_robot, ref_robot, real_to_sim=False)
pybullet_robots = bind_robot(ScalerManipulator, Pybullet)
pybullet_robots_2 = bind_robot(ScalerManipulator, Pybullet)

target_csv = FileOutput('test_robot.csv')       # save to test.csv at the same dir as the
ref_csv = FileOutput('ref_robot.csv')
arm2_csv = FileOutput('arm2_robot.csv')
pybullet_csv = FileOutput('pybullet_robot.csv')
pybullet_csv2 = FileOutput('pybullet_robot.csv')



# add robots to simulation


s.add_robot(ref_robot, (ref_csv,))
#s.add_robot(arm_2,(arm2_csv,))
#s.add_robot(target_robot, (target_csv,))
#s.add_robot(pybullet_robots, (pybullet_csv,))

#s.add_robot(pybullet_robots, (pybullet_csv2,))

# add process
#s.add_process(at_process)

s.run(max_duration=300, realtime=True)  # run 10sec, at the end of run, automatically do outputs.


#AutotunePlot(ref_csv.filepath, target_csv.filepath)
