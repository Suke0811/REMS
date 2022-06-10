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


pybullet_robots = bind_robot(ScalerManipulator, Pybullet)

pybullet_robots.init()
pybullet_robots.drive(pybullet_robots.inpt)
pybullet_robots.sense()
pybullet_robots.observe_state()
pass
