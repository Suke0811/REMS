from rems.robot_def.webots import YoubotArmDef, YoubotBaseDef
from rems.robot_def.webots import DifferentialDriveDef
from rems.robot_def.webots.DifferentialDriveDef import rule_to_nested_vel
from rems.typing.std.StdUnit import Pos, Vel, Ang, AngVel, AngAcc, UnitType, Percent
from rems.robots.differential_drive.CreateHard import CreateHard
from rems.robots.differential_drive.DynabotHard import DynabotHard
from rems.robots.differential_drive.WoodbotHard import WoodbotHard
from rems.typing import MapRule, DefDict


class YoubotDef(YoubotArmDef, YoubotBaseDef):
    def __init__(self, *args, **kwargs):
        YoubotArmDef.__init__(self, *args, **kwargs)
        YoubotBaseDef.__init__(self, *args, **kwargs)


        # YoubotArmDef.__init__(self, *args, **kwargs)
        # YoubotBaseDef.__init__(self, *args, **kwargs)

    def define(self, *args, **kwargs):
        YoubotArmDef.define(self, *args, **kwargs)
        YoubotBaseDef.define(self, *args, **kwargs)
        # YoubotArmDef.define(self, *args, **kwargs)
        # YoubotBaseDef.define(self, *args, **kwargs)
        self.name = 'youBot'

