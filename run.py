import logging, os, time
import pandas as pd
from sim.SimRay import Sim
from sim.inputs import FileInput
from sim.outputs import FileOutput
import matplotlib.pyplot as plt
from sim.utils.tictoc import tictoc
from sim.robots.bind_robot import bind_robot
from sim.robots.scalear_leg.ScalerManipulatorDef import ScalerManipulator
from sim.tuning.AutoTuning import AutoTuning
from sim.robots.scalear_leg.ScalarHard import ScalerHard
from sim.robots.bind.kinematic_model.KinematicModel import KinematicModel
from sim.utils.autotune_plots import AutotunePlot


PRINT = True

LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()

if PRINT:
    logging.basicConfig(level=LOGLEVEL)

s = Sim(DT=0.02)    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
i = FileInput('sim/utils/target_robot_circle_line.csv', loop=True)
#i = KeyboardInput()
#i = JoystickInput()



s.set_input(i)  # specify inputs to run

# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model

ref_robot = bind_robot(ScalerManipulator, ScalerHard, '/dev/ttyUSB0', 2)
target_robot = bind_robot(ScalerManipulator, KinematicModel)
at_process = AutoTuning(target_robot, ref_robot, real_to_sim=False)

target_csv = FileOutput('test_robot.csv')       # save to test.csv at the same dir as the
ref_csv = FileOutput('ref_robot.csv')

# add robots to simulation

s.add_robot(ref_robot, (ref_csv,))
s.add_robot(target_robot, (target_csv,))


# add process
#s.add_process(at_process)

s.run(max_duration=10, realtime=True)  # run 10sec, at the end of run, automatically do outputs.


#AutotunePlot(ref_csv.filepath, target_csv.filepath)
