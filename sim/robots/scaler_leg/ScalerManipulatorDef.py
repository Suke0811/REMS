from sim.robots.RobotDefBase import RobotDefBase
from sim.typing import DefDict, BindRule
from sim.typing.definitions import *
from sim.robots.scaler_leg.kinematics.SCALAR_kinematics import ScalerKinematics

class ScalerManipulator(RobotDefBase):
    NUM_JOINTS = 6

    def __init__(self):
        RobotDefBase.__init__(self)
        self.kin = ScalerKinematics()
        self.unit = 1000.0
        self.rule = BindRule(bind_from=POS_3D,
                             bind_func=lambda *xs: [x / self.unit for x in xs],
                             bind_to=POS_3D,
                             inv_bind_func=lambda *xs: [x * self.unit for x in xs])
        self.which_leg = 3

    def define(self, *args, **kwargs):
        # input is 2D pos
        self.inpt = DefDict(POS_3D)
        # state is joint pos and vel
        self.state = DefDict((POS_3D, VEL_POS_3D))
        # sensors are joins pos and vel
        self.outpt = DefDict((joint_pos(self.NUM_JOINTS), joint_vel(self.NUM_JOINTS)), prefixes=['j', 'd_j'])

        self.joint_space = DefDict(joint_pos(self.NUM_JOINTS))
        # Task space is 3D position and quaternion
        self.task_space = T_MAT
        # Jacobian is not available
        self.jacobian = None
        self.info = DefDict({'h2_norm':0, 'h2_norm_x':0, 'h2_norm_y':0})

    def fk(self, jointspace: DefDict, *args, **kwargs):
        d = self.kin.scalar_forward_kinematics(which_leg=self.which_leg,
                                               joint_angles=self.joint_space.format(jointspace).list())
        return self.rule.bind(self.task_space.format(d))

    def ik(self, taskspace: DefDict, *args, **kwargs):
        self.task_space.data = dict(r11=1, r22=-1, r33=-1,
                                    r12=0, r13=0, r21=0, r23=0, r31=0, r32=0)
        t = self.task_space.format(taskspace)
        t.set(self.rule.inv_bind(t))
        return self.joint_space.format(
            self.kin.scalar_inverse_kinematics(which_leg=self.which_leg,
                                               T_shi_wrist3=t.as_ruled(),
                                               is_first_ik=all([i == 0.0 for i in self.joint_space.list()]),
                                               prev_angles=self.joint_space.list()))

    def calc_vel(self, pre_state, curr_state):
        dx = (curr_state[0] - pre_state[0]) / self.run.DT
        dy = (curr_state[1] - pre_state[1]) / self.run.DT
        dz = (curr_state[2] - pre_state[2]) / self.run.DT
        self.state.data = {'d_x': dx, 'd_y': dy, 'd_z': dz}
