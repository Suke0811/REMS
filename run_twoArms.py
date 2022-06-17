import logging, os, time
import pandas as pd
from sim.Sim import Sim
from sim.inputs import FileInput
from sim.outputs import FileOutput
import matplotlib.pyplot as plt
from sim.utils.tictoc import tictoc
from sim.robots.bind_robot import bind_robot
from sim.robots.scalear_leg.ScalerManipulatorDef import ScalerManipulator
from sim.tuning.AutoTuning import AutoTuning
from sim.robots.scalear_leg.ScalarHard import ScalerHard
from sim.robots.bind.kinematic_model.KinematicModel import KinematicModel
import numpy as np

PRINT = True

LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()

if PRINT:
    logging.basicConfig(level=LOGLEVEL)

s = Sim(DT=0.25)    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
i1 = FileInput('test_robot.csv', loop=True)
i2 = FileInput('ref_robot.csv', loop=True)
#i = KeyboardInput()
#i = JoystickInput()

s.set_input(i1)  # specify inputs to run

# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model
ref_robot = bind_robot(ScalerManipulator, ScalerHard, '/dev/ttyUSB0')


#at_process = AutoTuning(ref_robot1, ref_robot2, real_to_sim=False)

target_csv = FileOutput('test_robot.csv')       # save to test.csv at the same dir as the
ref_csv = FileOutput('ref_robot.csv')

# add robots to simulation
s.add_robot(ref_robot1, (ref_csv,))
s.add_robot(ref_robot2, (target_csv,))

# add process
#s.add_process(at_process)

s.run(max_duration=150, realtime=True)  # run 10sec, at the end of run, automatically do outputs.
