from rems.typing.definitions import *


# 1 or 2 legs, 6 DoF, w/o body, Fixed -> manipulator
# 4 legs, 3 DoF, w/ or w/o body, Fixed/Mobile  ->  Walking
# 4 legs, 6 DoF, w/ or w/o body,  Fixed/Mobile  ->  6 Dof
# 4 legs, 6+1Dof, w/ or w/o body, Fixed/Mobile  ->  Climbing

class ModeBase:
    NAME = None
    ACTIVE_LEGs = [0, 1, 2, 3]
    DOF = 6
    MOBILE = False
    BODY_JOINT = True
    URDF_PATH = None
    MESH_DIR = None
    NAME_LEG = 'leg'
    NAME_BODY_JOINT = 'bj.0'

    def __init__(self):
        self.NUM_JOINT = len(self.ACTIVE_LEGs) * self.DOF

    def state(self):    # Body, leg pos
        state = DefDict((POS_3D, VEL_POS_3D))
        return state

    def inpt(self):     # Legs and body joint angle
        DEF = [{self.NAME_BODY_JOINT: float}, define(self.NAME_LEG, self.ACTIVE_LEGs, DefDict(POS_3D))]
        prefixes = [self.NAME_BODY_JOINT, self.NAME_LEG]

        inpt = DefDict(tuple(DEF), prefixes=prefixes, suffixes=POS_3D)
        return inpt

    def outpt(self):    # all joint pos and vel in a flat structure
        DEF = [joint_pos(self.NUM_JOINT)]
        DEF_d = [joint_vel(self.NUM_JOINT)]
        prefixes = ['j', 'd_j']

        DEF.append({self.NAME_BODY_JOINT: float})
        DEF_d.append({'d_'+self.NAME_BODY_JOINT: float})
        prefixes.append(self.NAME_BODY_JOINT)
        prefixes.append('d_'+self.NAME_BODY_JOINT)
        DEF.extend(DEF_d)
        outpt = DefDict(tuple(DEF), prefixes=prefixes)
        return outpt

    def jointspace(self):   # Leg and j0-j6
        DEF = [{self.NAME_BODY_JOINT: float}, define(self.NAME_LEG, self.ACTIVE_LEGs, DefDict(joint_pos(self.DOF), prefixes='j'))]
        prefixes = [self.NAME_BODY_JOINT, self.NAME_LEG]

        jointspace = DefDict(tuple(DEF), prefixes=prefixes)
        return jointspace

    def taskspace(self):    # T_mat for each leg
        taskspace = DefDict(define(self.NAME_LEG, self.ACTIVE_LEGs, T_MAT), prefixes=[self.NAME_LEG])
        return taskspace

class ScalerMode:
    class Walking(ModeBase):
        NAME = 'Walking'
        URDF_PATH = 'rems/robots/scaler/pybullet/urdf_scalar/urdf/scalar.urdf'
        MESH_DIR = 'rems/robots/scaler/pybullet/urdf_scalar/meshes/'
        DOF = 3
        def __init__(self, mobile=True):
            self.MOBILE = mobile
            super().__init__()

    class Manipulator(ModeBase):
        NAME = 'Manipulator'
        URDF_PATH = 'rems/robots/scaler/pybullet/urdf_scalar_6DoF/urdf/SCALAR_6DoF.urdf'
        MESH_DIR = 'rems/robots/scaler/pybullet/urdf_scalar_6DoF/meshes/'
        BODY_JOINT = False
        MOBILE = False
        def __init__(self, sync_both_leg=True):    # if not sync_both_leg, then each leg needs input
            self.ACTIVE_LEGs = [0, 1, 2, 3]
            if sync_both_leg:
                pass
            super().__init__()

        def jointspace(self):  # Leg and j0-j6
            DEF = [{self.NAME_BODY_JOINT: float},
                   define(self.NAME_LEG, self.ACTIVE_LEGs, DefDict(joint_pos(self.DOF), prefixes='j'), link=True)]
            prefixes = [self.NAME_BODY_JOINT, self.NAME_LEG]

            jointspace = DefDict(tuple(DEF), prefixes=prefixes)
            return jointspace

    class SixDoF(ModeBase):
        NAME = 'SixDoF'
        URDF_PATH = 'rems/robots/scaler/pybullet/urdf_scalar_6DoF/urdf/SCALAR_6DoF.urdf'
        MESH_DIR = 'rems/robots/scaler/pybullet/urdf_scalar_6DoF/meshes/'
        def __init__(self, mobile=True):
            self.MOBILE = mobile
            super().__init__()

    class Climbing(ModeBase):
        NAME = 'Climbing'
        def __init__(self, mobile=True):
            self.MOBILE = mobile
            super().__init__()

pass
