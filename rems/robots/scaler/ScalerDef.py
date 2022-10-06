from rems.robots.RobotDefBase import RobotDefBase
from rems.typing import DefDict, MapRule
from rems.typing.definitions import *
from rems.robots.scaler.kinematics.SCALAR_kinematics import ScalerKinematics
from rems.robots.scaler import ScalerMode, ModeBase

class ScalerDef(RobotDefBase):
    def __init__(self, mode: ModeBase=ScalerMode.Walking):
        RobotDefBase.__init__(self)
        self.mode = mode
        self.kin = ScalerKinematics()
        self.unit = 1000.0
        # a rule to convert meter to mm, and inverse of it
        self.rule = MapRule(origin=POS_3D,
                             func=lambda *xs: [x / self.unit for x in xs],
                             target=POS_3D,
                             inv_func=lambda *xs: [x * self.unit for x in xs], to_list=True)

    def define(self, *args, **kwargs):
        # input is 2D pos
        self.inpt = self.mode.inpt()
        # state is joint pos and vel
        self.state = self.mode.state()
        # sensors are joins pos and vel
        self.outpt = self.mode.outpt()

        self.joint_space = self.mode.jointspace()
        # Task space is 3D position and quaternion
        self.task_space = self.mode.taskspace()
        # Jacobian is not available
        self.jacobian = None
        #self.info = DefDict({'h2_norm':0, 'h2_norm_x':0, 'h2_norm_y':0})

    def fk(self, jointspace: DefDict, *args, **kwargs):
        if self.mode.NAME_LEG in jointspace.prefixes:
            j = self.joint_space.format(jointspace)
        else:
            j = self.joint_space.format(jointspace).leg(self.mode.ACTIVE_LEGs[0])
        t = self.task_space.clone()
        for leg, t_leg, j_leg in zip(self.mode.ACTIVE_LEGs, t.leg(), j.leg()):
            t_leg.set(self.kin.scalar_forward_kinematics(which_leg=leg,
                                               joint_angles=j_leg.list(),
                                                   with_body=self.mode.BODY_JOINT,
                                                   body_angle=j.bj()))
            t_leg.bind(self.rule)
        return t


    def ik(self, taskspace: DefDict, *args, **kwargs):
        if self.mode.NAME_LEG in taskspace.prefixes:
            t = self.task_space.format(taskspace)
        else:
            t = self.task_space.format(taskspace).leg(self.mode.ACTIVE_LEGs[0])
        j = self.joint_space.clone()
        for leg, t_leg, j_leg in zip(self.mode.ACTIVE_LEGs, t.leg(), j.leg()):
            t_leg.set(dict(r11=1, r22=-1, r33=-1,
                                    r12=0, r13=0, r21=0, r23=0, r31=0, r32=0))
            t_leg.inv_bind(self.rule)
            j_leg.set(self.kin.scalar_inverse_kinematics(which_leg=leg,
                                                   target_T=t_leg.as_ruled(),
                                                   is_first_ik=all(
                                                       [i == 0.0 for i in j_leg.list()]),
                                                   prev_angles=j_leg.list()))
        return j

    def calc_vel(self, pre_state, curr_state):
        dx = (curr_state[0] - pre_state[0]) / self.run.DT
        dy = (curr_state[1] - pre_state[1]) / self.run.DT
        dz = (curr_state[2] - pre_state[2]) / self.run.DT
        self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}
