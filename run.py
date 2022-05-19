import logging, os, time
import pandas as pd
from sim.Simulation import Sim
from sim.inputs import FileInput
from sim.outputs import FileOutput
from sim.robots import RobotDefBase
import matplotlib.pyplot as plt
from sim.utils.tictoc import tictoc

PRINT = True

LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()

if PRINT:
    logging.basicConfig(level=LOGLEVEL)

s = Sim()    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
#i = FileInput('target_robot.csv',loop=True)
#i = KeyboardInput()
i = JoystickInput()

s.set_input(i)  # specify inputs to run

# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model

ref_robot = WebotsModel()

target_csv = FileOutput('target_robot.csv')       # save to test.csv at the same dir as the

# add robots to simulation
s.add_robot(ref_robot, (ref_csv,))

@tictoc
s.run(max_duration=150, realtime=True)  # run 10sec, at the end of run, automatically do outputs.

