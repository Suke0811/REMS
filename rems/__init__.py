# for shorter path import
from .typing import DefDict, MapRule, UnitType
from .typing.std import StdUnit, StdDefinitions



# configs
from .Config import *
from .Operator import Operator


from os.path import dirname
from os.path import realpath
from os.path import relpath

from ._version import __version__
del _version

__author__ = 'UCLA LEMUR'
__credits__ = 'The Laboratory for Embedded Machines and Ubiquitous Robots'
__url__ = 'https://git.uclalemur.com/'
__description__ = 'Robotics Educational Middleware System'

REMS_DIR = dirname(realpath(__file__))

def remspath(f):
    return relpath(f, REMS_DIR)
