from .SCALER_v2_Leg_6DOF_gripper import Leg
import numpy as np
from rems.robots.scaler_leg.kinematics import util
from rems.robots.scaler_leg.kinematics.wrap_to_pi import wrap_to_pi



class ScalerKinematics(object):
    def __init__(self):
        self.k_model = Leg()

    def scalar_forward_kinematics(self, which_leg, joint_angles):
        #inputs:
        #which_leg -> leg index 0-3
        #joint angles -> [shoulder angle, q11, q21, wrist1, wrist2, wrist3]

        #output:
        #matrix T of wrist3 in shoulder frame

        T_0_shi = self.k_model.leg_fk_direct_calculation(0.0, joint_angles, which_leg, which_link=1, use_quaternion = False)
        T_shi_0 = np.linalg.inv(T_0_shi)
        T_0_wrist3 = self.k_model.leg_fk_direct_calculation(0.0, joint_angles, which_leg, which_link=9, use_quaternion = False)
        T_shi_wrist3 = np.dot(T_shi_0, T_0_wrist3)
        return T_shi_wrist3

    def scalar_inverse_kinematics(self, which_leg, T_shi_wrist3, is_first_ik=True, prev_angles=None):
        #inputs:
        #which_leg -> leg index 0-3
        #T_shi_wrist3 -> matrix T of wrist3 in shoulder frame
        #is_first_ik : if you don't have previous angles, set it to be True, else set it to be False
        #              if there's no previous angles, there would be some restriction for the situation: the initial position of the end effector can not be folded back to the shoulder, q11 -90 to 90 degrees and q21 0 to 180 degrees
        #prev_angles -> [shoulder angle, q11, q21, wrist1, wrist2, wrist3]*4 legs, totally a list of 24 numbers

        #output:
        #joint angles -> [shoulder angle, q11, q21, wrist1, wrist2, wrist3]

        quaternion_state = util.rotation_matrix_2_quaternion(T_shi_wrist3[0:3,0:3])

        shoulder_angle, q11, q12, q13, q21, q22, qw1, qw2, qw3, phi = self.k_model.leg_ik_direct_calculation_6DoF(T_shi_wrist3[0:3,3].tolist(), quaternion_state, which_leg, is_first_ik = is_first_ik, prev_angles = prev_angles)

        return wrap_to_pi([shoulder_angle, q11, q21, qw1, qw2, qw3])
        
