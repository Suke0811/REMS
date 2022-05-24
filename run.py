import logging, os, time
import pandas as pd
from sim.Sim import Sim
from sim.inputs import FileInput, KeyboardInput
from sim.outputs import FileOutput
from sim.robots import RobotDefBase
import matplotlib.pyplot as plt
from sim.utils.tictoc import tictoc
from sim.robots.bind_robot import bind_robot
from sim.robots.scalear_leg.ScalerManipulatorDef import ScalerManipulator
from sim.robots.scalear_leg.ScalarHard import ScalerHard
from sim.robots.NopRobot import NopRobot

PRINT = True

LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()

if PRINT:
    logging.basicConfig(level=LOGLEVEL)

s = Sim()    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
i = FileInput('target_robot.csv', loop=True)
#i = KeyboardInput()
#i = JoystickInput()

s.set_input(i)  # specify inputs to run

# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model

ref_robot = bind_robot(ScalerManipulator, ScalerHard, 'COM5')

target_csv = FileOutput('test_robot.csv')       # save to test.csv at the same dir as the

# add robots to simulation
s.add_robot(ref_robot, (target_csv,))

s.run(max_duration=6, realtime=True)  # run 10sec, at the end of run, automatically do outputs.


