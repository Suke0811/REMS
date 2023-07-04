from rems.robot_def.webots import YoubotArmDef, YoubotBaseDef


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

