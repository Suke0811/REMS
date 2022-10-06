__author__ = "Feng Xu, Zachary Lacey"
__email__ = "xufengmax@g.ucla.edu, zlacey1234@gmail.com"
__copyright__ = "Copyright 2021 RoMeLa"
__date__ = "October 7, 2021"

__version__ = "0.1.0"
__status__ = "Prototype"

import numpy as np

from .utils_SCALER import ScalerStandardUtilMethods
from rems.robots.scaler_leg.kinematics import util
from .hardware_constants import consts
from .wrap_to_pi import wrap_to_pi
from scipy.optimize import fsolve

# robot_name = "SCALER_climbing_6DoF"
#data_name = glob.read_one_data(glob.mem_settings, 'robot_name')
#robot_name = list_name_robots[int(data_name)]
robot_name = 'SCALER_climbing_6DoF_gripper'
robot_consts = consts[robot_name]

scaler_std_utils = ScalerStandardUtilMethods

# The Z Offset from the Shoulder Servo to the Top Leg Servo. [units: mm]
TOP_LEG_SERVO_OFF_Z = robot_consts.TOP_LEG_SERVO_OFF_Z

BOTTOM_LEG_SERVO_OFF_Z = robot_consts.BOTTOM_LEG_SERVO_OFF_Z

# Center Point Between the Top Leg Servo and the Bottom Leg Servo. (These X and Z represent the Position (X,Z) of the
# Parallel Leg Origin with respect to Shoulder's Reference Frame) [units: mm]
LEG_ORIGIN_X = robot_consts.LEG_ORIGIN_X
LEG_ORIGIN_Z = robot_consts.LEG_ORIGIN_Z

# Rotation Angle Offset of the Parallel Leg Structure (Based on the X and Z Offset Positions of the Top and Bottom Leg
# Servos). [units: radians]
LEG_ROT_OFF_ANGLE = robot_consts.LEG_ROT_OFF_ANGLE

# Length of the Leg Link 1 (Links from the Top/Bottom Leg Servos to the Elbow Joints). [units: mm]
L_LEG_LINK_1 = robot_consts.L_LEG_LINK_1

# Length of the Leg Link 2 (Links from the Elbow Joints to the Wrist Servo). [units: mm]
L_LEG_LINK_2 = robot_consts.L_LEG_LINK_2

L_LEG_LINK_2_NEW = robot_consts.L_LEG_LINK_2_NEW

# Length of the Wrist Link [units: mm]
L_WRIST = robot_consts.L_WRIST

# Length of the Gripper offset (x-component) from the Spherical Joint [units: mm]
L_GRIPPER_OFFSET_X = robot_consts.L_GRIPPER_OFFSET_X

# Length of the Gripper offset (y-component) from the Spherical Joint [units: mm]
L_GRIPPER_OFFSET_Y = robot_consts.L_GRIPPER_OFFSET_Y

# Length Between the Leg Servos (Servo Pair) [units: mm]
L_BLSP = robot_consts.L_BLSP

# Length of the Battery Link [units: mm]
L_BATTERY = robot_consts.L_BATTERY

# Length of the Body Link [units: mm]
L_BL = robot_consts.L_BL


L_LEG_LINK_A23_WRIST = robot_consts.L_LEG_LINK_A23_WRIST

L_LEG_LINK_A22_WRIST = robot_consts.L_LEG_LINK_A22_WRIST

LEG_GAMMA_OFF_ANGLE = robot_consts.LEG_GAMMA_OFF_ANGLE

LEG_THETA_1_OFF_ANGLE = robot_consts.LEG_THETA_1_OFF_ANGLE


T_wrist_gripper_0and2 = robot_consts.T_wrist_gripper_0and2

T_wrist_gripper_1and3 = robot_consts.T_wrist_gripper_1and3

T_wrist_gripper_0and2_inv = robot_consts.T_wrist_gripper_0and2_inv

T_wrist_gripper_1and3_inv = robot_consts.T_wrist_gripper_1and3_inv

# State Dimension of each Footstep.
DIM_FOOTSTEP = robot_consts.DIM_FOOTSTEP

# gripper hardware constants
IJ_X = robot_consts.L_GRIPPER_2D_L_IJ_X
IJ_Y = robot_consts.L_GRIPPER_2D_L_IJ_Y

L1 = robot_consts.L_GRIPPER_2D_L1
L2 = robot_consts.L_GRIPPER_2D_L2
L3 = robot_consts.L_GRIPPER_2D_L3
L4 = robot_consts.L_GRIPPER_2D_L4
L5 = robot_consts.L_GRIPPER_2D_L5
L6 = robot_consts.L_GRIPPER_2D_L6
L7 = robot_consts.L_GRIPPER_2D_L7
L8 = robot_consts.L_GRIPPER_2D_L8
L9 = robot_consts.L_GRIPPER_2D_L9
L10 = robot_consts.L_GRIPPER_2D_L10
L11 = robot_consts.L_GRIPPER_2D_L11
L12 = robot_consts.L_GRIPPER_2D_L12
L13 = robot_consts.L_GRIPPER_2D_L13
L14 = robot_consts.L_GRIPPER_2D_L14
L15 = robot_consts.L_GRIPPER_2D_L15
scaler_helper = scaler_std_utils()
class Leg:
    """ Class: Leg
    This Class represents the kinematics of each Leg (The SCALER v2 has four legs). It specifically holds the forward
    and inverse kinematics function for the parallel legs.
    """

    @staticmethod
    def leg_fk_direct_calculation(body_angle, theta_angles, which_leg, which_link=-1, use_quaternion = True):
        """ Calculate the Full Forward Kinematics of the Parallel Legs
        This method calculates the full forward kinematics of the parallel legs for SCALER_v2 (6-DoF Climbing
        Configuration w/o Gripper Kinematics). This assumes that the full forward kinematics specifies the position of
        the center of the gripper and the orientation of the gripper (quaternion) with respect to SCALER_v2's Body
        Reference Frame.
        Args:
            body_angle: This is the angle of the body posture motor (i.e., MOTOR_ID = 0 for SCALER_v2 (6-DoF Climbing
                        Configuration w/o Gripper Kinematics)) [units: radians]
            theta_angles: This is a [1 x 3] Matrix which contains the direct joint angles of a single SCALER_v2 leg. It
                          is important to note that these angles are directly measured from the leg reference frame.
                          More specifically, they are the active joint angles (NOT in the motor frame)
                theta_angles = [shoulder_angle, q11, q21]
                    shoulder_angle  = Angle of the Shoulder Joint (i.e., MOTOR_ID = 1, 4, 7, 10)
                                      [units: radians]
                    q11             = Angle of the Top Leg Servo Joint (i.e., MOTOR_ID = 2, 5, 8, 11)
                                      [units: radians]
                    q21             = Angle of the Bottom Leg Servo Joint (i.e., MOTOR_ID = 3, 6, 9, 12)
                                      [units: radians]
                    qw1             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 13, 16, 19, 22)
                                      [units: radians]
                    qw2             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 14, 17, 20, 23)
                                      [units: radians]
                    qw3             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 15, 18, 21, 24)
                                      [units: radians]
            which_leg: This is an index variable that specifies which leg we want to calculate the forward kinematics.
                For SCALER_v2, we have four legs in total. This means that which_leg has the following options
                which_leg = 0: Leg 1 (Front Right Leg).
                which_leg = 1: Leg 2 (Back Right Leg).
                which_leg = 2: Leg 3 (Back Left Leg).
                which_leg = 3: Leg 4 (Front Left Leg).


            which_link: This is an index variable that specifies which link we want to calculate the forward kinematics. All the frames are from body frame
                which_link = 1: shoulder frame before rotation of shoulder angle
                which_link = 2: frame A after rotation of q11
                which_link = 3: frame F after rotation of q21
                which_link = 4: frame B after rotation of q12
                which_link = 5: frame E after rotation of q22
                which_link = 6: frame C after rotation of q13
                which_link = 7: frame wrist1 after rotation of qw1
                which_link = 8: frame wrist2 after rotation of qw2
                which_link = 9: frame wrist3 after rotation of qw3
                which_link = 11: frame wrist1 without rotation from frame E
                (by default) all the other which_link: frame gripper_center

            use_quaternion: (by default)If it is True, the final result of frame gripper_center would be in the format of [posx,posy,posz, qw, qx, qy, qz], but all the other returned links would be T matrix
                            If it is False, all the outputs would be T matrix
                            

                


        Returns:
            state: The State of the specified leg for SCALER_v2. This is a [7 x 1] Matrix that contains the XYZ
                   Position of a single leg's footstep (center of gripper) and the Orientation (quaternion) of the
                   gripper with respect to the body frame.
                state[0]: X Position of the footstep with respect to the body frame. [units: mm]
                state[1]: Y Position of the footstep with respect to the body frame. [units: mm]
                state[2]: Z Position of the footstep with respect to the body frame. [units: mm]
                state[3]: W (Scalar) Orientation of the gripper (quaternion) with respect to the body frame.
                state[4]: X Orientation of the gripper (quaternion) with respect to the body frame.
                state[5]: Y Orientation of the gripper (quaternion) with respect to the body frame.
                state[6]: Z Orientation of the gripper (quaternion) with respect to the body frame.

            or T matrix(depands on use_quaternion)

        """
        # Unpack the theta angles
        # Unpack the theta Angles
        shoulder_angle = theta_angles[0]
        q11 = theta_angles[1]
        q21 = theta_angles[2]
        th4 = theta_angles[3]
        th5 = theta_angles[4]
        th6 = theta_angles[5]

        # TODO: Eventually, the State Dimension should be state = np.zeros([DIM_FOOTSTEP, 1])
        state = np.zeros([3, 1])

        shoulder_vertex = scaler_std_utils.find_shoulder_vertices(body_angle,
                                                                  use_find_specific_shoulder=True,
                                                                  which_leg=which_leg)

        
        if which_leg==0 or which_leg==3:
            T_0_shi = np.array([[np.cos(np.pi/2), -np.sin(np.pi/2), 0, 0],
                                [np.sin(np.pi/2),  np.cos(np.pi/2), 0, 0],
                                [            0,              0, 1, 0],
                                [0,0,0,1]])
        else:
            T_0_shi = np.array([[np.cos(-np.pi/2), -np.sin(-np.pi/2), 0, 0],
                                [np.sin(-np.pi/2),  np.cos(-np.pi/2), 0, 0],
                                [            0,              0, 1, 0],
                                [0,0,0,1]])
        T_0_shi[0:3,3] = np.array(shoulder_vertex)       
        if which_link==1:
            return T_0_shi


        T_shi_A_rot = np.array([[np.cos(shoulder_angle), -np.sin(shoulder_angle), 0, 0],
                               [np.sin(shoulder_angle), np.cos(shoulder_angle),  0, 0],
                               [                     0,                       0, 1, 0],
                               [0,0,0,1]])

        T_shi_A_t = np.array([[1, 0, 0, LEG_ORIGIN_X],
                              [0, 1, 0, 0],
                              [0, 0, 1, TOP_LEG_SERVO_OFF_Z],
                              [0,0,0,1]])

        T_shi_A = np.dot(T_shi_A_rot, T_shi_A_t)

        #if which_link==100:
        #    return np.dot(T_0_shi, T_shi_A_rot)


        T_shi_A_cord_rot_1 = np.array([[1,                 0,                 0, 0],
                                       [0,   np.cos(np.pi/2),  -np.sin(np.pi/2), 0],
                                       [0,   np.sin(np.pi/2),   np.cos(np.pi/2), 0],
                                       [0,0,0,1]])

        T_shi_A_cord_rot_2 = np.array([[np.cos(-np.pi/2),  -np.sin(-np.pi/2),  0, 0],
                                       [np.sin(-np.pi/2),   np.cos(-np.pi/2),  0, 0],
                                       [               0,                  0,  1, 0],
                                       [0,0,0,1]])
        T_shi_A_cord_rot = np.dot(T_shi_A_cord_rot_1, T_shi_A_cord_rot_2)



        T_0_A = np.dot(np.dot(T_0_shi, T_shi_A), T_shi_A_cord_rot)

        if which_link==2:
            T_A_rot = np.array([[np.cos(q11),  -np.sin(q11),  0, 0],
                                [np.sin(q11),   np.cos(q11),  0, 0],
                                [               0,                  0,  1, 0],
                                [0,0,0,1]])
            return np.dot(T_0_A, T_A_rot)

        T_0_F = np.array(T_0_A)
        T_0_F[2,3] = T_0_F[2,3] - (TOP_LEG_SERVO_OFF_Z - BOTTOM_LEG_SERVO_OFF_Z)

        if which_link==3:
            T_F_rot = np.array([[np.cos(q21),  -np.sin(q21),  0, 0],
                                [np.sin(q21),   np.cos(q21),  0, 0],
                                [               0,                  0,  1, 0],
                                [0,0,0,1]])
            return np.dot(T_0_F, T_F_rot)

        
        #leg_origin_vertex = T_0_A[0:3,3]


        fk_variable_0, fk_variable_1, fk_variable_2, fk_variable_3, L_EB, GEC_ANGLE = Leg.find_fk_variables(q11, q21)

        if which_link==4 or which_link==6:
            COS_BCE = (- L_EB**2 + L_LEG_LINK_2_NEW**2 + L_LEG_LINK_2**2) / (2*L_LEG_LINK_2*L_LEG_LINK_2_NEW)
            SIN_BCE = np.sqrt(1 - COS_BCE**2)
            BCE_ANGLE = np.arctan2(SIN_BCE, COS_BCE)
            
            HCE_ANGLE = np.pi/2 - GEC_ANGLE
            BCH_ANGLE = BCE_ANGLE - HCE_ANGLE

            T_A_B = np.array([[np.cos(BCH_ANGLE), -np.sin(BCH_ANGLE),0,L_LEG_LINK_1 * np.cos(q11)],
                              [np.sin(BCH_ANGLE),  np.cos(BCH_ANGLE),0,L_LEG_LINK_1 * np.sin(q11)],
                              [0,0,1,0],
                              [0,0,0,1]])
            T_0_B = np.dot(T_0_A, T_A_B)

            if which_link == 4:
                return T_0_B
            else:
                DCJ_ANGLE = np.pi - BCE_ANGLE - (np.pi/2 - LEG_THETA_1_OFF_ANGLE)
                T_B_C = np.array([[np.cos(DCJ_ANGLE), -np.sin(DCJ_ANGLE),0,L_LEG_LINK_2],
                                  [np.sin(DCJ_ANGLE),  np.cos(DCJ_ANGLE),0,0],
                                  [0,0,1,0],
                                  [0,0,0,1]])
                T_0_C = np.dot(T_0_B, T_B_C)
                return T_0_C


        GET_ANGLE = GEC_ANGLE + LEG_THETA_1_OFF_ANGLE
        GET_ANGLE_OFF = np.pi/2 - GET_ANGLE
        T_A_E = np.array([[np.cos(-GET_ANGLE_OFF), -np.sin(-GET_ANGLE_OFF),0,L_LEG_LINK_1 * np.cos(q21)+L_BLSP],
                          [np.sin(-GET_ANGLE_OFF),  np.cos(-GET_ANGLE_OFF),0,L_LEG_LINK_1 * np.sin(q21)],
                          [0,0,1,0],
                          [0,0,0,1]])
        T_0_E = np.dot(T_0_A, T_A_E)
        if which_link==5:
            return T_0_E


        #T_A_TOE = np.array([[1,0,0,fk_variable_3],
        #                    [0,1,0,fk_variable_2],
        #                    [0,0,1,0],
        #                    [0,0,0,1]])

        #T_0_TOE = np.dot(T_0_A, T_A_TOE)

        #state[0] = leg_origin_vertex[0] + np.cos(shoulder_angle) * fk_variable_2

        #state[1] = leg_origin_vertex[1] + np.sin(shoulder_angle) * fk_variable_2

        #state[2] = leg_origin_vertex[2] - fk_variable_3

        T_E_wrist = np.array([[1,0,0,L_LEG_LINK_A22_WRIST],
                              [0,1,0,0],
                              [0,0,1,0],
                              [0,0,0,1]])
        T_0_wrist = np.dot(T_0_E, T_E_wrist)


        if which_link==11:
            return T_0_wrist


        T_wrist_1 = np.array([ [ np.cos(th4),  -np.sin(th4),      0,         0],
                               [           0,             0,      1,         0],
                               [-np.sin(th4),  -np.cos(th4),      0,         0],
                               [           0,             0,      0,         1] ])

        T_0_wrist_1 = np.dot(T_0_wrist,T_wrist_1)

        if which_link==7:
            return T_0_wrist_1


        T_wrist_2 = np.array([ [ np.cos(th5 + np.pi/2),  -np.sin(th5 + np.pi/2),      0,         0],
                               [                     0,                       0,      1,         0],
                               [-np.sin(th5 + np.pi/2),  -np.cos(th5 + np.pi/2),      0,         0],
                               [                     0,                       0,      0,         1] ])

        T_0_wrist_2 = np.dot(T_0_wrist_1,T_wrist_2)

        if which_link==8:
            return T_0_wrist_2

        T_wrist_3 = np.array([ [ np.cos(th6 - np.pi/2),  -np.sin(th6 - np.pi/2),      0,         0],
                               [                     0,                       0,     -1,         0],
                               [ np.sin(th6 - np.pi/2),   np.cos(th6 - np.pi/2),      0,         0],
                               [                     0,                       0,      0,         1] ])

        T_0_wrist_3 = np.dot(T_0_wrist_2,T_wrist_3)

        if which_link==9:
            return T_0_wrist_3

        if which_leg == 0 or which_leg == 2:
            T_0_gripper_center =  np.dot(T_0_wrist_3, T_wrist_gripper_0and2)
        else:
            T_0_gripper_center =  np.dot(T_0_wrist_3, T_wrist_gripper_1and3)

        if use_quaternion == True:
            state = np.zeros([7, 1])
            quaternion_state = util.rotation_matrix_2_quaternion(T_0_gripper_center[0:3,0:3])
            state[0:3,0] = T_0_gripper_center[0:3,3]
            state[3:7,0] = quaternion_state
            return state
        else:
            return T_0_gripper_center

    @staticmethod
    def leg_gripper_fk(body_angle, theta_angles, L_actuator, theta_actuator, which_leg):
        """ Calculate the Full Forward Kinematics of the Parallel Legs
                This method calculates the full forward kinematics of the parallel legs for SCALER_v2 (6-DoF Climbing
                Configuration with Gripper Kinematics). This assumes that the full forward kinematics specifies the position of
                the gripper fingertips and the orientation of the fingertip orientation (euler) with respect to SCALER_v2's Body
                Reference Frame.
                Args:
                    body_angle: This is the angle of the body posture motor (i.e., MOTOR_ID = 0 for SCALER_v2 (6-DoF Climbing
                                Configuration w/o Gripper Kinematics)) [units: radians]
                    theta_angles: This is a [1 x 3] Matrix which contains the direct joint angles of a single SCALER_v2 leg. It
                                  is important to note that these angles are directly measured from the leg reference frame.
                                  More specifically, they are the active joint angles (NOT in the motor frame)
                        theta_angles = [shoulder_angle, q11, q21]
                            shoulder_angle  = Angle of the Shoulder Joint (i.e., MOTOR_ID = 1, 4, 7, 10)
                                              [units: radians]
                            q11             = Angle of the Top Leg Servo Joint (i.e., MOTOR_ID = 2, 5, 8, 11)
                                              [units: radians]
                            q21             = Angle of the Bottom Leg Servo Joint (i.e., MOTOR_ID = 3, 6, 9, 12)
                                              [units: radians]
                            qw1             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 13, 16, 19, 22)
                                              [units: radians]
                            qw2             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 14, 17, 20, 23)
                                              [units: radians]
                            qw3             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 15, 18, 21, 24)
                                              [units: radians]
                    L_actuator: This is the length of the linear actuator of the gripper (i.e., MOTOR_ID = ....), scalar
                    value [units: mm]
                    theta_actuator: This is the offset angle specified by the hinge on the gripper, scalar value
                    [units: radians]
                    which_leg: This is an index variable that specifies which leg we want to calculate the forward kinematics.
                        For SCALER_v2, we have four legs in total. This means that which_leg has the following options
                        which_leg = 0: Leg 1 (Front Right Leg).
                        which_leg = 1: Leg 2 (Back Right Leg).
                        which_leg = 2: Leg 3 (Back Left Leg).
                        which_leg = 3: Leg 4 (Front Left Leg).
                Returns:
                    gripper_center_body_toe_M: Finger 1 vector in 3D-space (X, Y, Z, w, x, y, z) from the Body to
                    the desired Finger Tip Positions and quaternion Rotation (Finger 1 or Finger M). [dim: 7 x 1] [units: mm]
                """
        # We calculate the FK of the 6 DoF leg (outputs the position and quaterninon orientation of the center of gripper)
        state_gripper_center = Leg.leg_fk_direct_calculation(body_angle,theta_angles,which_leg)

        # We get the center of gripper position
        x_pos,y_pos,z_pos = state_gripper_center[0], state_gripper_center[1], state_gripper_center[2]
        # We convert quaterninon orientation to 3x3 rotation matrix
        w,x_rot,y_rot,z_rot = state_gripper_center[3], state_gripper_center[4], state_gripper_center[5], state_gripper_center[6]
        R_BC = util.quaternion_2_rotation_matrix(np.array([w,x_rot,y_rot,z_rot]))
        # Create T matrix that specifies the frame from center of gripper to body
        T_BC = np.array([[R_BC[0,0],R_BC[0,1],R_BC[0,2],x_pos],
                        [R_BC[1,0],R_BC[1,1],R_BC[1,2],y_pos],
                        [R_BC[2,0],R_BC[2,1],R_BC[2,2],z_pos],
                        [0,0,0,1]], dtype=np.float32)

        # We get the T matrix for fingertip position and orientation of fingertip 1 and fingertip 2 in
        # center of gripper frame
        gripper_center_origin_toe_M, gripper_center_origin_toe_N = \
            scaler_std_utils.gripper_2d_two_finger_fk_noOffset(T_BC,L_actuator,theta_actuator,which_leg)
        # TODO: eventually we should switch which local gripper FK IK be used in 7 DoF FK IK
        # gripper_center_origin_toe_M, gripper_center_origin_toe_N = \
        #     scaler_std_utils.gripper_2d_two_finger_fk(T_BC,L_actuator,theta_actuator,which_leg)

        return gripper_center_origin_toe_M, gripper_center_origin_toe_N

    @staticmethod
    def leg_gripper_ik(gripper_center_body_toe_M, gripper_center_body_toe_N, body_angle, which_leg=-1):
        """ Inverse Kinematics of the Two Finger 2D Gripper
                Args:
                    gripper_center_body_toe_M: Finger 1 vector in 3D-space (X, Y, Z, w, x, y, z) from the Body to the desired Finger Tip
                                             Positions and quaternion rotation (Finger 1 or Finger M). [dim: 7 x 1] [units: mm]
                        gripper_center_body_toe_M = [finger1_X, finger1_Y, finger1_Z, finger2_w, finger2_x, finger2_y, finger2_z]
                    gripper_center_body_toe_M: Finger 2 vector in 3D-space (X, Y, Z) from the Body to the desired Finger Tip
                                            Positions (Finger 2 or Finger N). [dim: 7 x 1] [units: mm]
                        gripper_center_body_toe_M = [finger2_X, finger2_Y, finger2_Z, finger2_w, finger2_x, finger2_y, finger2_z]
                    body_angle: This is the angle of the body posture motor (i.e., MOTOR_ID = 0 for SCALER_v2) [units: radians]
                    which_leg: This is an index variable that specifies which leg we want to calculate the forward kinematics.
                        For SCALER_v2, we have four legs in total. This means that which_leg has the following options
                        which_leg = 0: Leg 1 (Front Right Leg).
                        which_leg = 1: Leg 2 (Back Right Leg).
                        which_leg = 2: Leg 3 (Back Left Leg).
                        which_leg = 3: Leg 4 (Front Left Leg).
        """
        # We calculate the linear actuator and gripper offset angle value (i.e., IK of gripper only)
        # TODO: eventually we should switch which local gripper FK IK be used in 7 DoF FK IK
        L_actuator, theta_actuator, T_Gripper_Center_Body, _, _ = scaler_std_utils.gripper_2d_two_finger_ik_noOffset(gripper_center_body_toe_M,gripper_center_body_toe_N)


        #shoulder_vertex = Scaler_utils.ScalerStandardUtilMethods.find_shoulder_vertices(body_angle,
        #                                                                                use_find_specific_shoulder=True,
        #                                                                                which_leg=which_leg)

        #shoulder_2_toe_xyz = T_Gripper_Center_Body[0:3,3]-shoulder_vertex
        #wrist_quaternion = util.rotation_matrix_2_quaternion(T_Gripper_Center_Body[0:3,0:3])
        T_0_shi = Leg.leg_fk_direct_calculation(body_angle, [0,0,0,0,0,0], which_leg, which_link=1, use_quaternion = False)
        T_shi_0 = np.linalg.inv(T_0_shi)
        T_shi_gripper = np.dot(T_shi_0, T_Gripper_Center_Body)
        shoulder_2_toe_xyz = T_shi_gripper[0:3,3]
        wrist_quaternion = util.rotation_matrix_2_quaternion(T_shi_gripper[0:3,0:3])

        if which_leg == -1:
            [shoulder_angle, q11, q12, q13, q21, q22, qw1, qw2, qw3, phi] = \
                Leg.leg_ik_direct_calculation(shoulder_2_toe_xyz, wrist_quaternion)
        else:
            [shoulder_angle, q11, q12, q13, q21, q22, qw1, qw2, qw3, phi] = \
                Leg.leg_ik_direct_calculation(shoulder_2_toe_xyz, wrist_quaternion, which_leg=which_leg)

        joint_angles = [shoulder_angle, q11, q21, qw1, qw2, qw3]


        return [joint_angles, L_actuator, theta_actuator]

    @staticmethod
    def gripper_2d_two_finger_fk(T_shoulder_gripper_center, L_actuator, theta_actuator, which_leg):
        """
        """
        # Unpack the 2D Gripper Hardware Constants (Link Lengths)
        L_GRIPPER_2D_L_IJ_X = robot_consts.L_GRIPPER_2D_L_IJ_X
        L_GRIPPER_2D_L_IJ_Y = robot_consts.L_GRIPPER_2D_L_IJ_Y

        L_GRIPPER_2D_L1 = robot_consts.L_GRIPPER_2D_L1

        L_GRIPPER_2D_L2 = robot_consts.L_GRIPPER_2D_L2
        L_GRIPPER_2D_L3 = robot_consts.L_GRIPPER_2D_L3
        L_GRIPPER_2D_L4 = robot_consts.L_GRIPPER_2D_L4
        L_GRIPPER_2D_L5 = robot_consts.L_GRIPPER_2D_L5
        L_GRIPPER_2D_L6 = robot_consts.L_GRIPPER_2D_L6
        L_GRIPPER_2D_L7 = robot_consts.L_GRIPPER_2D_L7

        L_GRIPPER_2D_L8 = robot_consts.L_GRIPPER_2D_L8
        L_GRIPPER_2D_L9 = robot_consts.L_GRIPPER_2D_L9
        L_GRIPPER_2D_L10 = robot_consts.L_GRIPPER_2D_L10
        L_GRIPPER_2D_L11 = robot_consts.L_GRIPPER_2D_L11
        L_GRIPPER_2D_L12 = robot_consts.L_GRIPPER_2D_L12
        L_GRIPPER_2D_L13 = robot_consts.L_GRIPPER_2D_L13

        L_GRIPPER_2D_L14 = robot_consts.L_GRIPPER_2D_L14
        L_GRIPPER_2D_L15 = robot_consts.L_GRIPPER_2D_L15

        T_gripper_center_gripper_origin = np.array([[0.0, 0.0, -1.0, 0.0],
                                                    [0.0, 1.0, 0.0, 0.0],
                                                    [1.0, 0.0, 0.0, -L_GRIPPER_2D_L1],
                                                    [0.0, 0.0, 0.0, 1.0]])

        x_D = L_actuator * np.cos(theta_actuator)
        y_D = L_actuator * np.sin(theta_actuator)

        # Calculating theta3, theta4, given x_D and y_D
        r_BD = np.sqrt((x_D - L_GRIPPER_2D_L1) ** 2 + (y_D - L_GRIPPER_2D_L2) ** 2)

        theta3 = np.arccos((r_BD ** 2 + L_GRIPPER_2D_L3 ** 2 - L_GRIPPER_2D_L4 ** 2) / (2 * r_BD * L_GRIPPER_2D_L3)) + \
                 np.arctan2(y_D - L_GRIPPER_2D_L2, x_D - L_GRIPPER_2D_L1)

        theta4 = theta3 + \
                 np.arccos((L_GRIPPER_2D_L3 ** 2 + L_GRIPPER_2D_L4 ** 2 - r_BD ** 2) /
                           (2 * L_GRIPPER_2D_L3 * L_GRIPPER_2D_L4)) - np.pi

        # Calculating theta6, theta5, given x_D and y_D
        r_FD = np.sqrt((x_D - L_GRIPPER_2D_L1) ** 2 + (y_D + L_GRIPPER_2D_L7) ** 2)

        theta6 = np.arctan2(y_D + L_GRIPPER_2D_L7, x_D - L_GRIPPER_2D_L1) - \
                 np.arccos((r_FD ** 2 + L_GRIPPER_2D_L6 ** 2 - L_GRIPPER_2D_L5 ** 2) / (2 * r_FD * L_GRIPPER_2D_L6))

        theta5 = np.pi - \
                 np.arccos((L_GRIPPER_2D_L6 ** 2 + L_GRIPPER_2D_L5 ** 2 - r_FD ** 2) /
                           (2 * L_GRIPPER_2D_L6 * L_GRIPPER_2D_L5)) + theta6

        # Calculating x_G, y_G, given theta3, and theta4 (Due to closed chain constraint)
        x_G = L_GRIPPER_2D_L1 + L_GRIPPER_2D_L3 * np.cos(theta3) + L_GRIPPER_2D_L13 * np.cos(theta4 - np.pi)
        y_G = L_GRIPPER_2D_L2 + L_GRIPPER_2D_L3 * np.sin(theta3) + L_GRIPPER_2D_L13 * np.sin(theta4 - np.pi)

        # Calculating theta11, theta12, given x_G and y_G
        r_IG = np.sqrt((x_G - L_GRIPPER_2D_L_IJ_X) ** 2 + (y_G - L_GRIPPER_2D_L_IJ_Y / 2) ** 2)

        theta11 = np.arccos((r_IG ** 2 + L_GRIPPER_2D_L11 ** 2 - L_GRIPPER_2D_L12 ** 2) /
                            (2 * r_IG * L_GRIPPER_2D_L11)) + \
                  np.arctan2(y_G - L_GRIPPER_2D_L_IJ_Y / 2, x_G - L_GRIPPER_2D_L_IJ_X)

        theta12 = theta11 + np.arccos((L_GRIPPER_2D_L11 ** 2 + L_GRIPPER_2D_L12 ** 2 - r_IG ** 2) /
                                      (2 * L_GRIPPER_2D_L11 * L_GRIPPER_2D_L12)) - np.pi

        # Calculating x_L, y_L, given theta6, and theta5 (Due to closed chain constraint)
        x_L = L_GRIPPER_2D_L1 + L_GRIPPER_2D_L6 * np.cos(theta6) + L_GRIPPER_2D_L10 * np.cos(theta5 - np.pi)
        y_L = -L_GRIPPER_2D_L7 + L_GRIPPER_2D_L6 * np.sin(theta6) + L_GRIPPER_2D_L10 * np.sin(theta5 - np.pi)

        # Calculating theta8, theta9, given x_L and y_L
        r_JK = np.sqrt((x_L - L_GRIPPER_2D_L_IJ_X) ** 2 + (y_L + L_GRIPPER_2D_L_IJ_Y / 2) ** 2)

        theta8 = np.arctan2(y_L + L_GRIPPER_2D_L_IJ_Y / 2, x_L - L_GRIPPER_2D_L_IJ_X) - \
                 np.arccos((r_JK ** 2 + L_GRIPPER_2D_L8 ** 2 - L_GRIPPER_2D_L9 ** 2) / (2 * r_JK * L_GRIPPER_2D_L8))

        theta9 = np.pi - \
                 np.arccos((L_GRIPPER_2D_L8 ** 2 + L_GRIPPER_2D_L9 ** 2 - r_JK ** 2) /
                           (2 * L_GRIPPER_2D_L8 * L_GRIPPER_2D_L9)) + theta8

        # Assign equivalent theta angles
        theta10 = theta5
        theta13 = theta4
        theta14 = theta12
        theta15 = theta9

        T_shoulder_gripper_origin = np.dot(T_shoulder_gripper_center, T_gripper_center_gripper_origin)

        # Position of Point M (One of the Finger Tips)
        x_M = L_GRIPPER_2D_L_IJ_X + L_GRIPPER_2D_L11 * np.cos(theta11) + \
              (L_GRIPPER_2D_L12 + L_GRIPPER_2D_L14) * np.cos(theta12)

        y_M = L_GRIPPER_2D_L_IJ_Y / 2 + L_GRIPPER_2D_L11 * np.sin(theta11) + \
              (L_GRIPPER_2D_L12 + L_GRIPPER_2D_L14) * np.sin(theta12)

        # Position of Point N (the other Finger Tip)
        x_N = L_GRIPPER_2D_L_IJ_X + L_GRIPPER_2D_L8 * np.cos(theta8) + \
              (L_GRIPPER_2D_L9 + L_GRIPPER_2D_L15) * np.cos(theta9)

        y_N = - L_GRIPPER_2D_L_IJ_Y / 2 + L_GRIPPER_2D_L8 * np.sin(theta8) + \
              (L_GRIPPER_2D_L9 + L_GRIPPER_2D_L15) * np.sin(theta9)

        T_gripper_origin_toe_M = np.array([[np.cos(theta12), -np.sin(theta12), 0, x_M],
                                           [np.sin(theta12), np.cos(theta12), 0, y_M],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]])

        T_gripper_origin_toe_N = np.array([[np.cos(theta9), -np.sin(theta9), 0, x_N],
                                           [np.sin(theta9), np.cos(theta9), 0, y_N],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]])

        T_shoulder_toe_M = np.dot(T_shoulder_gripper_origin, T_gripper_origin_toe_M)
        T_shoulder_toe_N = np.dot(T_shoulder_gripper_origin, T_gripper_origin_toe_N)

        R_M = T_shoulder_toe_M[0:3, 0:3]
        R_M_quat = util.rotation_matrix_2_quaternion(R_M)
        P_M = T_shoulder_toe_M[:, 3]
        R_N = T_shoulder_toe_N[0:3, 0:3]
        R_N_quat = util.rotation_matrix_2_quaternion(R_N)
        P_N = T_shoulder_toe_N[:, 3]

        fingertip1 = np.array([P_M[0], P_M[1], P_M[2], R_M_quat[0], R_M_quat[1], R_M_quat[2], R_M_quat[3]])
        fingertip2 = np.array([P_N[0], P_N[1], P_N[2], R_N_quat[0], R_N_quat[1], R_N_quat[2], R_N_quat[3]])

        return fingertip1.reshape(7, 1), fingertip2.reshape(7, 1)

    @staticmethod
    def gripper_2d_two_finger_ik(T_Toe1, T_Toe2):
        """ Inverse Kinematics of the Two Finger 2D Gripper
                Args:
                    gripper_center_body_toe_M: Finger 1 vector in 3D-space (X, Y, Z, w, x, y, z) from the Body to the desired Finger Tip
                                             Positions and quaternion rotation (Finger 1 or Finger M). [dim: 7 x 1] [units: mm]
                        gripper_center_body_toe_M = [finger1_X, finger1_Y, finger1_Z, finger2_w, finger2_x, finger2_y, finger2_z]
                    gripper_center_body_toe_M: Finger 2 vector in 3D-space (X, Y, Z) from the Body to the desired Finger Tip
                                            Positions (Finger 2 or Finger N). [dim: 1 x 3] [units: mm]
                        gripper_center_body_toe_M = [finger2_X, finger2_Y, finger2_Z, finger2_w, finger2_x, finger2_y, finger2_z]
        """

        IJ_X = robot_consts.L_GRIPPER_2D_L_IJ_X
        IJ_Y = robot_consts.L_GRIPPER_2D_L_IJ_Y

        L1 = robot_consts.L_GRIPPER_2D_L1
        L2 = robot_consts.L_GRIPPER_2D_L2
        L3 = robot_consts.L_GRIPPER_2D_L3
        L4 = robot_consts.L_GRIPPER_2D_L4
        L5 = robot_consts.L_GRIPPER_2D_L5
        L6 = robot_consts.L_GRIPPER_2D_L6
        L7 = robot_consts.L_GRIPPER_2D_L7
        L8 = robot_consts.L_GRIPPER_2D_L8
        L9 = robot_consts.L_GRIPPER_2D_L9
        L10 = robot_consts.L_GRIPPER_2D_L10
        L11 = robot_consts.L_GRIPPER_2D_L11
        L12 = robot_consts.L_GRIPPER_2D_L12
        L13 = robot_consts.L_GRIPPER_2D_L13
        L14 = robot_consts.L_GRIPPER_2D_L14
        L15 = robot_consts.L_GRIPPER_2D_L15

        P_M_Body = T_Toe1[0:3].reshape(3, 1)
        P_N_Body = T_Toe2[0:3].reshape(3, 1)

        Body_R_ToeM = util.quaternion_2_rotation_matrix(T_Toe1[3:].reshape(4, ))
        Body_R_ToeN = util.quaternion_2_rotation_matrix(T_Toe2[3:].reshape(4, ))

        #Body_R_ToeM = T_Toe1[0:3, 0:3]
        #Body_R_ToeN = T_Toe2[0:3, 0:3]
        #P_M_Body = np.array(T_Toe1[0:3, 3]).reshape(3, 1)
        #P_N_Body = np.array(T_Toe2[0:3, 3]).reshape(3, 1)

        P_G_Body = P_M_Body - np.dot(Body_R_ToeM, np.array([L14, 0, 0]).reshape(3, 1))
        P_H_Body = P_M_Body - np.dot(Body_R_ToeM, np.array([L14 + L12, 0, 0]).reshape(3, 1))

        P_L_Body = P_N_Body - np.dot(Body_R_ToeN, np.array([L15, 0, 0]).reshape(3, 1))
        P_K_Body = P_N_Body - np.dot(Body_R_ToeN, np.array([L15 + L9, 0, 0]).reshape(3, 1))

        r_GL_mag = np.linalg.norm(P_G_Body - P_L_Body)
        r_GD_mag = L13 + L4
        r_LD_mag = L10 + L5
        r_MG_mag = L14
        r_NL_mag = L15
        r_ML_mag = np.linalg.norm(P_M_Body - P_L_Body)
        r_NG_mag = np.linalg.norm(P_N_Body - P_G_Body)

        angle_GDL = np.arccos((r_GD_mag ** 2 + r_LD_mag ** 2 - r_GL_mag ** 2) / (2 * r_LD_mag * r_GD_mag))
        angle_LGD = (np.pi - angle_GDL) / 2
        angle_GLD = angle_LGD

        angle_MGL = np.arccos((r_GL_mag ** 2 + r_MG_mag ** 2 - r_ML_mag ** 2) / (2 * r_GL_mag * r_MG_mag))
        angle_NLG = np.arccos((r_GL_mag ** 2 + r_NL_mag ** 2 - r_NG_mag ** 2) / (2 * r_GL_mag * r_NL_mag))

        angle_MGD = angle_MGL + angle_LGD
        angle_NLD = angle_NLG + angle_GLD

        R_G_C = np.array([[np.cos(np.pi - angle_MGD), -np.sin(np.pi - angle_MGD), 0],
                          [np.sin(np.pi - angle_MGD), np.cos(np.pi - angle_MGD), 0],
                          [0, 0, 1]])

        R_L_E = np.array([[np.cos(angle_NLD - np.pi), -np.sin(angle_NLD - np.pi), 0],
                          [np.sin(angle_NLD - np.pi), np.cos(angle_NLD - np.pi), 0],
                          [0, 0, 1]])

        Body_R_C = np.dot(Body_R_ToeM, R_G_C)

        Body_R_E = np.dot(Body_R_ToeN, R_L_E)
        Body_R_H = Body_R_ToeM
        Body_R_K = Body_R_ToeN

        P_C_Body = P_G_Body - np.dot(Body_R_C, np.array([L13, 0, 0]).reshape(3, 1))
        P_E_Body = P_L_Body - np.dot(Body_R_E, np.array([L10, 0, 0]).reshape(3, 1))

        def kinematicConstraint(theta, *args):
            P_C_body, body_R_C, P_E_body, body_R_E, P_H_body, body_R_H, P_K_body, body_R_K = args
            R_C_B = np.array([[np.cos(theta[0]), np.sin(theta[0]), 0],
                              [-np.sin(theta[0]), np.cos(theta[0]), 0],
                              [0, 0, 1]])

            P_B_Body = P_C_body - np.dot(np.dot(body_R_C, R_C_B), np.array([L3, 0, 0]).reshape(3, 1))

            R_E_F = np.array([[np.cos(theta[1]), -np.sin(theta[1]), 0],
                              [np.sin(theta[1]), np.cos(theta[1]), 0],
                              [0, 0, 1]])

            P_F_Body = P_E_body - np.dot(np.dot(body_R_E, R_E_F), np.array([L6, 0, 0]).reshape(3, 1))

            R_H_I = np.array([[np.cos(theta[2]), np.sin(theta[2]), 0],
                              [-np.sin(theta[2]), np.cos(theta[2]), 0],
                              [0, 0, 1]])

            P_I_Body = P_H_body + np.dot(np.dot(body_R_H, R_H_I), np.array([L11, 0, 0]).reshape(3, 1))

            R_K_J = np.array([[np.cos(theta[3]), -np.sin(theta[3]), 0],
                              [np.sin(theta[3]), np.cos(theta[3]), 0],
                              [0, 0, 1]])

            P_J_Body = P_K_body + np.dot(np.dot(body_R_K, R_K_J), np.array([L8, 0, 0]).reshape(3, 1))

            r_BI = P_I_Body - P_B_Body
            r_IJ = P_J_Body - P_I_Body
            r_IF = P_F_Body - P_I_Body
            r_BJ = P_J_Body - P_B_Body
            r_JF = P_F_Body - P_J_Body

            BI_IJ = np.cross(r_BI, r_IJ, axis=0)
            BI_IF = np.cross(r_BI, r_IF, axis=0)

            BJ_JF = np.cross(r_BJ, r_JF, axis=0)
            IJ_JF = np.cross(r_IJ, r_JF, axis=0)

            r_IJ_mag = np.linalg.norm(r_IJ)
            r_BI_mag = np.linalg.norm(r_BI)
            r_IF_mag = np.linalg.norm(r_IF)
            r_BJ_mag = np.linalg.norm(r_BJ)
            r_JF_mag = np.linalg.norm(r_JF)

            F = np.array([r_BI_mag - (L2 - IJ_Y / 2),
                          r_IF_mag - (L7 + IJ_Y / 2),
                          r_BJ_mag - (L2 + IJ_Y / 2),
                          r_JF_mag - (L7 - IJ_Y / 2)])
            return F

        theta_init = []
        theta_init.append(np.array([np.pi / 4, np.pi / 4, 3 * np.pi / 4, 3 * np.pi / 4]))
        theta_init.append(np.array([np.pi / 4, np.pi / 2, np.pi / 2, np.pi / 4]))
        theta_init.append(np.array([np.pi / 2, np.pi / 4, np.pi / 4, np.pi / 2]))

        args = (P_C_Body, Body_R_C, P_E_Body, Body_R_E, P_H_Body, Body_R_H, P_K_Body, Body_R_K)

        theta_fsolve = []
        for i, th in enumerate(theta_init):
            theta_fsolve.append(fsolve(kinematicConstraint, th, args=args, xtol=0.00001))

        theta = theta_fsolve[1]
        for th in theta_fsolve:
            if any(t < 0 for t in th):
                continue
            theta = th



        """
        th0 = np.array([np.pi / 4, np.pi / 4, 3 * np.pi / 4, 3 * np.pi / 4])
        #    th = kinematicConstraint(th0,P_C_Body, Body_R_C, P_E_Body, Body_R_E, P_H_Body, Body_R_H, P_K_Body, Body_R_K )
        args = (P_C_Body, Body_R_C, P_E_Body, Body_R_E, P_H_Body, Body_R_H, P_K_Body, Body_R_K)
        theta = fsolve(kinematicConstraint, th0, args=args, xtol=0.004)
        """

        R_C_B = np.array([[np.cos(theta[0]), np.sin(theta[0]), 0],
                          [-np.sin(theta[0]), np.cos(theta[0]), 0],
                          [0, 0, 1]])

        P_B_Body = P_C_Body - np.dot(np.dot(Body_R_C, R_C_B), np.array([L3, 0, 0]).reshape(3, 1))

        R_E_F = np.array([[np.cos(theta[1]), -np.sin(theta[1]), 0],
                          [np.sin(theta[1]), np.cos(theta[1]), 0],
                          [0, 0, 1]])

        P_F_Body = P_E_Body - np.dot(np.dot(Body_R_E, R_E_F), np.array([L6, 0, 0]).reshape(3, 1))

        R_H_I = np.array([[np.cos(theta[2]), np.sin(theta[2]), 0],
                          [-np.sin(theta[2]), np.cos(theta[2]), 0],
                          [0, 0, 1]])

        P_I_Body = P_H_Body + np.dot(np.dot(Body_R_H, R_H_I), np.array([L11, 0, 0]).reshape(3, 1))

        R_K_J = np.array([[np.cos(theta[3]), -np.sin(theta[3]), 0],
                          [np.sin(theta[3]), np.cos(theta[3]), 0],
                          [0, 0, 1]])

        P_J_Body = P_K_Body + np.dot(np.dot(Body_R_K, R_K_J), np.array([L8, 0, 0]).reshape(3, 1))

        r_CF = P_F_Body - P_C_Body

        r_CF_mag = np.linalg.norm(r_CF)

        angle_CBF = np.arccos(
            (L3 ** 2 + (L2 + L7) ** 2 - r_CF_mag ** 2) / (2 * L3 * (L2 + L7)))

        R_B_A = np.array([[np.cos(angle_CBF - np.pi / 2), np.sin(angle_CBF - np.pi / 2), 0],
                          [-np.sin(angle_CBF - np.pi / 2), np.cos(angle_CBF - np.pi / 2), 0],
                          [0, 0, 1]])

        Body_R_Wrist = np.dot(Body_R_C, np.dot(R_C_B, R_B_A))

        P_A_Body = (P_B_Body - P_F_Body) / 2 + P_F_Body

        P_Gripper_Origin_Body = P_A_Body - np.dot(Body_R_Wrist, np.array([L1, 0, 0]).reshape(3, 1))

        P_D_Body = P_G_Body - np.dot(Body_R_C, np.array([L13 + L4, 0, 0]).reshape(3, 1))

        r_BD = np.dot(Body_R_Wrist.transpose(), (P_D_Body - P_Gripper_Origin_Body))

        L_actuator = np.linalg.norm(r_BD)

        theta_actuator = np.arctan2(r_BD[1], r_BD[0])

        T_Gripper_Origin_Body = np.array([[Body_R_Wrist[0,0],Body_R_Wrist[0,1],Body_R_Wrist[0,2],P_Gripper_Origin_Body[0][0]],
                                          [Body_R_Wrist[1,0],Body_R_Wrist[1,1],Body_R_Wrist[1,2],P_Gripper_Origin_Body[1][0]],
                                          [Body_R_Wrist[2,0],Body_R_Wrist[2,1],Body_R_Wrist[2,2],P_Gripper_Origin_Body[2][0]],
                                          [0,0,0,1]])


        T_gripper_center_gripper_origin_inv = np.array([[  0.0,   0.0,   1.0, L1],
                                                        [  0.0,   1.0,   0.0,   0.0],
                                                        [ -1.0,  0.0,  0.0,  0.0],
                                                        [  0.0,   0.0,   0.0,   1.0]])

        T_Gripper_Center_Body = np.dot(T_Gripper_Origin_Body,T_gripper_center_gripper_origin_inv)



        return L_actuator, theta_actuator[0], T_Gripper_Center_Body

    @staticmethod
    def leg_fk_position_direct_calculation(body_angle, theta_angles, which_leg):
        """ Calculate the Full Forward Kinematics (Position) of the Parallel Legs
        This method calculates the full forward kinematics (position) of the parallel legs for SCALER_v2 (6-DoF
        Climbing Configuration w/o Gripper Kinematics). This assumes that the full forward kinematics specifies the
        position of the center of the gripper with respect to SCALER_v2's Body Reference Frame.
        Args:
            body_angle: This is the angle of the body posture motor (i.e., MOTOR_ID = 0 for SCALER_v2 (6-DoF Climbing
                        Configuration w/o Gripper Kinematics)) [units: radians]
            theta_angles: This is a [1 x 3] Matrix which contains the direct joint angles of a single SCALER_v2 leg. It
                          is important to note that these angles are directly measured from the leg reference frame.
                          More specifically, they are the active joint angles (NOT in the motor frame)
                theta_angles = [shoulder_angle, q11, q21]
                    shoulder_angle  = Angle of the Shoulder Joint (i.e., MOTOR_ID = 1, 4, 7, 10)
                                      [units: radians]
                    q11             = Angle of the Top Leg Servo Joint (i.e., MOTOR_ID = 2, 5, 8, 11)
                                      [units: radians]
                    q21             = Angle of the Bottom Leg Servo Joint (i.e., MOTOR_ID = 3, 6, 9, 12)
                                      [units: radians]
                    qw1             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 13, 16, 19, 22)
                                      [units: radians]
                    qw2             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 14, 17, 20, 23)
                                      [units: radians]
                    qw3             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 15, 18, 21, 24)
                                      [units: radians]
            which_leg: This is an index variable that specifies which leg we want to calculate the forward kinematics.
                For SCALER_v2, we have four legs in total. This means that which_leg has the following options
                which_leg = 0: Leg 1 (Front Right Leg).
                which_leg = 1: Leg 2 (Back Right Leg).
                which_leg = 2: Leg 3 (Back Left Leg).
                which_leg = 3: Leg 4 (Front Left Leg).
        Returns:
            position_state: The State of the specified leg for SCALER_v2. This is a [3 x 1] Matrix that contains the
                            XYZ Position of a single leg's footstep (center of gripper) with respect to the body frame.
                position_state[0]: X Position of the footstep with respect to the body frame. [units: mm]
                position_state[1]: Y Position of the footstep with respect to the body frame. [units: mm]
                position_state[2]: Z Position of the footstep with respect to the body frame. [units: mm]
        """
        # Unpack the theta angles
        shoulder_angle = theta_angles[0]
        q11 = theta_angles[1]
        q21 = theta_angles[2]
        qw1 = theta_angles[3]
        qw2 = theta_angles[4]
        qw3 = theta_angles[5]

        position_state = np.zeros([3, 1])

        shoulder_vertex = scaler_std_utils.find_shoulder_vertices(body_angle,
                                                                  use_find_specific_shoulder=True,
                                                                  which_leg=which_leg)

        leg_origin_vertex = shoulder_vertex + np.array([LEG_ORIGIN_X * np.cos(shoulder_angle),
                                                        LEG_ORIGIN_X * np.sin(shoulder_angle),
                                                        TOP_LEG_SERVO_OFF_Z])

        fk_variable_0, fk_variable_1, fk_variable_2, fk_variable_3, fk_variable_4, fk_variable_5, fk_variable_6 = \
            Leg.find_fk_variables(q11, q21, qw1, qw2, qw3, which_leg)

        phi_v2 = np.arctan2(fk_variable_2 - L_LEG_LINK_1 * np.sin(q21),
                            fk_variable_3 - L_LEG_LINK_1 * np.cos(q21) - L_BLSP)

        position_state[0] = leg_origin_vertex[0] + np.cos(shoulder_angle) * fk_variable_2 + \
            np.cos(shoulder_angle) * np.sin(phi_v2) * fk_variable_4 + \
            np.cos(shoulder_angle) * np.cos(phi_v2) * fk_variable_5 + \
            np.sin(shoulder_angle) * fk_variable_6

        position_state[1] = leg_origin_vertex[1] + np.sin(shoulder_angle) * fk_variable_2 + \
            np.sin(shoulder_angle) * np.sin(phi_v2) * fk_variable_4 + \
            np.sin(shoulder_angle) * np.cos(phi_v2) * fk_variable_5 - \
            np.cos(shoulder_angle) * fk_variable_6

        position_state[2] = leg_origin_vertex[2] - fk_variable_3 - \
            np.cos(phi_v2) * fk_variable_4 + \
            np.sin(phi_v2) * fk_variable_5

        return position_state

    @staticmethod
    def leg_fk_orientation_direct_calculation(body_angle, theta_angles, which_leg):
        """ Calculate the Full Forward Kinematics (Orientation) of the Parallel Legs
        This method calculates the full forward kinematics (orientation) of the parallel legs for SCALER_v2 (6-DoF
        Climbing Configuration w/o Gripper Kinematics). This assumes that the full forward kinematics specifies the
        orientation of the gripper (quaternion) with respect to SCALER_v2's Body Reference Frame.
        Args:
            body_angle: This is the angle of the body posture motor (i.e., MOTOR_ID = 0 for SCALER_v2 (6-DoF Climbing
                        Configuration w/o Gripper Kinematics)) [units: radians]
            theta_angles: This is a [1 x 3] Matrix which contains the direct joint angles of a single SCALER_v2 leg. It
                          is important to note that these angles are directly measured from the leg reference frame.
                          More specifically, they are the active joint angles (NOT in the motor frame)
                theta_angles = [shoulder_angle, q11, q21]
                    shoulder_angle  = Angle of the Shoulder Joint (i.e., MOTOR_ID = 1, 4, 7, 10)
                                      [units: radians]
                    q11             = Angle of the Top Leg Servo Joint (i.e., MOTOR_ID = 2, 5, 8, 11)
                                      [units: radians]
                    q21             = Angle of the Bottom Leg Servo Joint (i.e., MOTOR_ID = 3, 6, 9, 12)
                                      [units: radians]
                    qw1             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 13, 16, 19, 22)
                                      [units: radians]
                    qw2             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 14, 17, 20, 23)
                                      [units: radians]
                    qw3             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 15, 18, 21, 24)
                                      [units: radians]
            which_leg: This is an index variable that specifies which leg we want to calculate the forward kinematics.
                For SCALER_v2, we have four legs in total. This means that which_leg has the following options
                which_leg = 0: Leg 1 (Front Right Leg).
                which_leg = 1: Leg 2 (Back Right Leg).
                which_leg = 2: Leg 3 (Back Left Leg).
                which_leg = 3: Leg 4 (Front Left Leg).
        Returns:
            state: The State of the specified leg for SCALER_v2. This is a [4 x 1] Matrix that contains the WXYZ
                   Orientation (quaternion) of the gripper with respect to the body frame.
                quaternion_state[0]: W (Scalar) Orientation of the gripper (quaternion) with respect to the body frame.
                quaternion_state[1]: X Orientation of the gripper (quaternion) with respect to the body frame.
                quaternion_state[2]: Y Orientation of the gripper (quaternion) with respect to the body frame.
                quaternion_state[3]: Z Orientation of the gripper (quaternion) with respect to the body frame.
        """
        # Unpack the theta angles
        shoulder_angle = theta_angles[0]
        q11 = theta_angles[1]
        q21 = theta_angles[2]
        qw1 = theta_angles[3]
        qw2 = theta_angles[4]
        qw3 = theta_angles[5]

        fk_variable_0, fk_variable_1, fk_variable_2, fk_variable_3, fk_variable_4, fk_variable_5, fk_variable_6 = \
            Leg.find_fk_variables(q11, q21, qw1, qw2, qw3, which_leg)

        phi_v2 = np.arctan2(fk_variable_2 - L_LEG_LINK_1 * np.sin(q21),
                            fk_variable_3 - L_LEG_LINK_1 * np.cos(q21) - L_BLSP)

        r_11 = np.cos(qw1) * np.cos(qw3) * np.sin(shoulder_angle) - \
            np.cos(phi_v2) * np.cos(shoulder_angle) * np.cos(qw2) * np.sin(qw3) + \
            np.cos(shoulder_angle) * np.cos(qw3) * np.sin(phi_v2) * np.sin(qw1) + \
            np.sin(shoulder_angle) * np.sin(qw1) * np.sin(qw2) * np.sin(qw3) - \
            np.cos(shoulder_angle) * np.cos(qw1) * np.sin(phi_v2) * np.sin(qw2) * np.sin(qw3)

        r_12 = np.cos(qw3) * np.sin(shoulder_angle) * np.sin(qw1) * np.sin(qw2) - \
            np.cos(phi_v2) * np.cos(shoulder_angle) * np.cos(qw2) * np.cos(qw3) - \
            np.cos(shoulder_angle) * np.sin(phi_v2) * np.sin(qw1) * np.sin(qw3) - \
            np.cos(qw1) * np.sin(shoulder_angle) * np.sin(qw3) - \
            np.cos(shoulder_angle) * np.cos(qw1) * np.cos(qw3) * np.sin(phi_v2) * np.sin(qw2)

        r_13 = np.cos(shoulder_angle) * np.cos(qw1) * np.cos(qw2) * np.sin(phi_v2) - \
            np.cos(phi_v2) * np.cos(shoulder_angle) * np.sin(qw2) - \
            np.cos(qw2) * np.sin(shoulder_angle) * np.sin(qw1)

        r_21 = np.cos(qw3) * np.sin(phi_v2) * np.sin(shoulder_angle) * np.sin(qw1) - \
            np.cos(phi_v2) * np.cos(qw2) * np.sin(shoulder_angle) * np.sin(qw3) - \
            np.cos(shoulder_angle) * np.cos(qw1) * np.cos(qw3) - \
            np.cos(shoulder_angle) * np.sin(qw1) * np.sin(qw2) * np.sin(qw3) - \
            np.cos(qw1) * np.sin(phi_v2) * np.sin(shoulder_angle) * np.sin(qw2) * np.sin(qw3)

        r_22 = np.cos(shoulder_angle) * np.cos(qw1) * np.sin(qw3) - \
            np.cos(phi_v2) * np.cos(qw2) * np.cos(qw3) * np.sin(shoulder_angle) - \
            np.cos(shoulder_angle) * np.cos(qw3) * np.sin(qw1) * np.sin(qw2) - \
            np.sin(phi_v2) * np.sin(shoulder_angle) * np.sin(qw1) * np.sin(qw3) - \
            np.cos(qw1) * np.cos(qw3) * np.sin(phi_v2) * np.sin(shoulder_angle) * np.sin(qw2)

        r_23 = np.cos(shoulder_angle) * np.cos(qw2) * np.sin(qw1) - \
            np.cos(phi_v2) * np.sin(shoulder_angle) * np.sin(qw2) + \
            np.cos(qw1) * np.cos(qw2) * np.sin(phi_v2) * np.sin(shoulder_angle)

        r_31 = np.cos(phi_v2) * np.cos(qw1) * np.sin(qw2) * np.sin(qw3) - \
            np.cos(phi_v2) * np.cos(qw3) * np.sin(qw1) - \
            np.cos(qw2) * np.sin(phi_v2) * np.sin(qw3)

        r_32 = np.cos(phi_v2) * np.sin(qw1) * np.sin(qw3) - \
            np.cos(qw2) * np.cos(qw3) * np.sin(phi_v2) + \
            np.cos(phi_v2) * np.cos(qw1) * np.cos(qw3) * np.sin(qw2)

        r_33 = - np.sin(phi_v2) * np.sin(qw2) - \
            np.cos(phi_v2) * np.cos(qw1) * np.cos(qw2)

        # Rotation Matrix of the Spherical Wrist Joint
        rotation_matrix_wrist = np.array([[r_11, r_12, r_13],
                                          [r_21, r_22, r_23],
                                          [r_31, r_32, r_33]])

        quaternion_state = util.rotation_matrix_2_quaternion(rotation_matrix_wrist)

        return quaternion_state

    @staticmethod
    def find_fk_variables(q11, q21):
        """ Calculates the Variables for forward kinematics

        Using this method removes many redundant calculations that were previously present and makes the forward
        kinematics method cleaner.

        Args:
            q11: Angle of the Top Leg Servo Joint (i.e., MOTOR_ID = 3, 4 | 9, 10 | 15, 16 | 21, 22) [units: radians]

            q21: Angle of the Bottom Leg Servo Joint (i.e., MOTOR_ID = 5, 6 | 11, 12 | 17, 18 | 23, 24) [units: radians]

        Returns:
            fk_variable_0, fk_variable_1, fk_variable_2, fk_variable_3: These are variables that describe the forward
                                                                        kinematics of SCALER_v2's Leg (parallel
                                                                        mechanism)

                fk_variable_0: This is the zeta_n variable

                fk_variable_1: This is the K variable

                fk_variable_2: This is the y-component of the end-effector point (y point of the end-effector Parallel
                               Leg).

                fk_variable_3: This is the x-component of the end-effector point (x point of the end-effector Parallel
                               Leg).

        """
        # Variable zeta_n
        fk_variable_0 = L_LEG_LINK_A22_WRIST / (2 * L_LEG_LINK_2)

        # Variable K
        fk_variable_1 = (L_LEG_LINK_A22_WRIST ** 2 / (
                L_BLSP ** 2 + 2 * L_BLSP * L_LEG_LINK_1 * (np.cos(q21) - np.cos(q11)) +
                2 * L_LEG_LINK_1 ** 2 * (1 - np.cos(q11 - q21))) - fk_variable_0 ** 2) ** (1 / 2)


        L_GB = L_LEG_LINK_1 * (np.cos(q11) - np.cos(q21)) - L_BLSP
        L_EG = L_LEG_LINK_1 * (np.sin(q21) - np.sin(q11))
        L_EB = np.sqrt(L_GB**2 + L_EG**2)

        COS_GEB = L_EG/L_EB
        SIN_GEB = L_GB/L_EB    

        COS_BEC = (L_EB**2 + L_LEG_LINK_2_NEW**2 - L_LEG_LINK_2**2) / (2*L_EB*L_LEG_LINK_2_NEW)
        SIN_BEC = np.sqrt(1 - COS_BEC**2)
  

        COS_GET = (COS_GEB*COS_BEC - SIN_GEB*SIN_BEC)*np.cos(LEG_THETA_1_OFF_ANGLE) - (SIN_GEB*COS_BEC + COS_GEB*SIN_BEC)*np.sin(LEG_THETA_1_OFF_ANGLE)
        SIN_GET = (SIN_GEB*COS_BEC + COS_GEB*SIN_BEC)*np.cos(LEG_THETA_1_OFF_ANGLE) + (COS_GEB*COS_BEC - SIN_GEB*SIN_BEC)*np.sin(LEG_THETA_1_OFF_ANGLE)


        L_EI = L_LEG_LINK_A22_WRIST * COS_GET
        L_IT = L_LEG_LINK_A22_WRIST * SIN_GET


        fk_variable_2 = L_LEG_LINK_1 * np.sin(q21) - L_EI
        fk_variable_3 = L_LEG_LINK_1 * np.cos(q21) + L_BLSP + L_IT

        GEC_ANGLE = np.arctan2(SIN_GEB, COS_GEB) + np.arctan2(SIN_BEC, COS_BEC)


        """
        # Variable y_L
        fk_variable_2 = (L_LEG_LINK_1 * (np.cos(q11) - np.cos(q21)) - L_BLSP) * \
                        (fk_variable_1 * np.cos(LEG_THETA_1_OFF_ANGLE) +
                         fk_variable_0 * np.sin(LEG_THETA_1_OFF_ANGLE)) - \
                        (L_LEG_LINK_1 * (np.sin(q21) - np.sin(q11))) * \
                        (fk_variable_0 * np.cos(LEG_THETA_1_OFF_ANGLE) -
                         fk_variable_1 * np.sin(LEG_THETA_1_OFF_ANGLE)) + L_LEG_LINK_1 * np.sin(q21)

        # Variable x_L
        fk_variable_3 = (L_LEG_LINK_1 * (np.sin(q21) - np.sin(q11))) * \
                        (fk_variable_1 * np.cos(LEG_THETA_1_OFF_ANGLE) +
                         fk_variable_0 * np.sin(LEG_THETA_1_OFF_ANGLE)) + \
                        (L_LEG_LINK_1 * (np.cos(q11) - np.cos(q21)) - L_BLSP) * \
                        (fk_variable_0 * np.cos(LEG_THETA_1_OFF_ANGLE) -
                         fk_variable_1 * np.sin(LEG_THETA_1_OFF_ANGLE)) + L_LEG_LINK_1 * np.cos(q21) + L_BLSP
        """

        return fk_variable_0, fk_variable_1, fk_variable_2, fk_variable_3, L_EB, GEC_ANGLE

    @staticmethod
    def leg_ik_direct_calculation(shoulder_2_toe_xyz, wrist_quaternion, which_leg=-1, is_first_ik= True, prev_angles = None):
        """ Calculates the Inverse Kinematics of the Parallel Leg and Wrist.
        This method calculates the inverse kinematics of the parallel leg for the SCALER_v2 given the position of
        the toe and the desired orientation of the spherical wrist (quaternion).

        verified joint angle range:
        shoulder_angle -> [-np.pi/3,np.pi/3]
                   q11 -> (np.pi*1.5,np.pi*2)
                   q21 -> [np.pi/4,np.pi/4*3]
                   qw1 -> [-np.pi/4,np.pi/4]
                   qw2 -> [-np.pi/4,np.pi/4]
                   qw3 -> [-np.pi/4,np.pi/4]

        Args:
            shoulder_2_toe_xyz: Vector in 3D-space (X, Y, Z) from the Shoulder Joint to the desired Toe Position.
                                [dim: 3 x 1][units: mm]
            wrist_quaternion: Quaternion (w, x, y, z) that represents the desired wrist orientation with respect to the
                              Body Frame of SCALER_v2. [dim: 1 x 4]
            which_leg (optional arg): This is an index variable that specifies which leg we want to find the shoulder
                                      vertex for. [default value: -1].
                For SCALER_v1, we have four legs in total. This means that which_leg has the following options
                which_leg = 0: Leg 1 (Front Right Leg).
                which_leg = 1: Leg 2 (Back Right Leg).
                which_leg = 2: Leg 3 (Back Left Leg).
                which_leg = 3: Leg 4 (Front Left Leg).
        Returns:
            shoulder_angle  : Angle of the Shoulder Joint [units: radians]
            q11             : Angle of the Top Leg Servo Joint [units: radians]
            q12             : Angle of the Top Leg Elbow Joint [units: radians] (would be always 0)
            q13             : Angle to satisfy five-bar closed chain (q13 = q21 + q22 - q11 - q12) [units: radians] (would be always 0)
            q21             : Angle of the Bottom Leg Servo Joint [units: radians]
            q22             : Angle of the Bottom Leg Elbow Joint [units: radians] (would be always 0)
            qw1             : Angle of the First Wrist Servo Joint [units: radians]
            qw2             : Angle of the Second Wrist Servo Joint [units: radians]
            qw3             : Angle of the Third Wrist Servo Joint [units: radians]
            phi             : Angle of Orientation Constraint of Parallel Leg Mechanism
                              (i.e., phi = q21 + q22 = q11 + q12 + q13) [units: radians] (would be always 0)
        """


        # Rotation Matrix of the desired wrist orientation with respect to SCALER v2 Body Reference Frame.
        rot_gripper = util.quaternion_2_rotation_matrix(wrist_quaternion)
        T_shoulder_gripper = np.eye(4, dtype=np.float32)
        T_shoulder_gripper[0:3,0:3] = rot_gripper
        T_shoulder_gripper[0:3,3] = np.array(shoulder_2_toe_xyz).reshape(-1)
        if which_leg == 0 or which_leg == 2:
            T_shoulder_wrist3 = np.dot(T_shoulder_gripper, T_wrist_gripper_0and2_inv)
        elif which_leg == 1 or which_leg == 3:
            T_shoulder_wrist3 = np.dot(T_shoulder_gripper, T_wrist_gripper_1and3_inv)
        else:
            print("Invalid leg index")
            return

        P_shoulder_wrist = T_shoulder_wrist3[0:3,3].reshape(-1)

        First_3_joint = Leg.leg_ik_direct_calculation_3DoF(P_shoulder_wrist, which_leg, is_first_ik, prev_angles)

        shoulder_angle = First_3_joint[0]
        q11 = First_3_joint[1]
        q21 = First_3_joint[2]

        if q11 < -np.pi/2:
            q11 = -np.pi-q11
        if q11 > np.pi/2:
            q11 = np.pi-q11
        if q21 < 0:
            q21 = -q21



        T_0_shoulder = Leg.leg_fk_direct_calculation(0.0, [shoulder_angle, q11, q21,0,0,0], 0, 1, use_quaternion = False)
        T_0_wrist = Leg.leg_fk_direct_calculation(0.0, [shoulder_angle, q11, q21,0,0,0], 0, 11, use_quaternion = False)
        T_shoulder_wrist = np.dot(np.linalg.inv(T_0_shoulder), T_0_wrist)

        T_wrist_wrist3 = np.dot(np.linalg.inv(T_shoulder_wrist), T_shoulder_wrist3)

        rot_wrist = T_wrist_wrist3[0:3,0:3]

        # Rotation Matrix of the wrist orientation with respect to the Wrist Reference Frame (before 3-DoF wrist
        # rotations). This can be thought of as the rotation matrix of the wrist (so we determine what the 3-DoF
        # spherical joint angle need to be).

        r_w_11, r_w_12, r_w_13, \
            r_w_21, r_w_22, r_w_23, \
            r_w_31, r_w_32, r_w_33 = util.unpack_rotation_matrix(rot_wrist)

        # Trivial solution to the Spherical Joint
        qw2 = np.arctan2(-r_w_23, np.sqrt(r_w_13 ** 2 + r_w_33 ** 2))
        qw1 = np.arctan2(- r_w_33 / np.cos(qw2),  r_w_13 / np.cos(qw2))
        qw3 = np.arctan2(- r_w_21 / np.cos(qw2), - r_w_22 / np.cos(qw2))

        return [shoulder_angle, q11, 0, 0, q21, 0, qw1, qw2, qw3, 0]

    @staticmethod
    def leg_ik_direct_calculation_6DoF(shoulder_2_toe_xyz, wrist_quaternion, which_leg=-1, is_first_ik= True, prev_angles = None):
        """ Calculates the Inverse Kinematics of the Parallel Leg and Wrist.
        This method calculates the inverse kinematics of the parallel leg for the SCALER_v2 given the position of
        the toe and the desired orientation of the spherical wrist (quaternion).

        verified joint angle range:
        shoulder_angle -> [-np.pi/3,np.pi/3]
                   q11 -> (np.pi*1.5,np.pi*2)
                   q21 -> [np.pi/4,np.pi/4*3]
                   qw1 -> [-np.pi/4,np.pi/4]
                   qw2 -> [-np.pi/4,np.pi/4]
                   qw3 -> [-np.pi/4,np.pi/4]

        Args:
            shoulder_2_toe_xyz: Vector in 3D-space (X, Y, Z) from the Shoulder Joint to the desired Toe Position.
                                [dim: 3 x 1][units: mm]
            wrist_quaternion: Quaternion (w, x, y, z) that represents the desired wrist orientation with respect to the
                              Body Frame of SCALER_v2. [dim: 1 x 4]
            which_leg (optional arg): This is an index variable that specifies which leg we want to find the shoulder
                                      vertex for. [default value: -1].
                For SCALER_v1, we have four legs in total. This means that which_leg has the following options
                which_leg = 0: Leg 1 (Front Right Leg).
                which_leg = 1: Leg 2 (Back Right Leg).
                which_leg = 2: Leg 3 (Back Left Leg).
                which_leg = 3: Leg 4 (Front Left Leg).
        Returns:
            shoulder_angle  : Angle of the Shoulder Joint [units: radians]
            q11             : Angle of the Top Leg Servo Joint [units: radians]
            q12             : Angle of the Top Leg Elbow Joint [units: radians] (would be always 0)
            q13             : Angle to satisfy five-bar closed chain (q13 = q21 + q22 - q11 - q12) [units: radians] (would be always 0)
            q21             : Angle of the Bottom Leg Servo Joint [units: radians]
            q22             : Angle of the Bottom Leg Elbow Joint [units: radians] (would be always 0)
            qw1             : Angle of the First Wrist Servo Joint [units: radians]
            qw2             : Angle of the Second Wrist Servo Joint [units: radians]
            qw3             : Angle of the Third Wrist Servo Joint [units: radians]
            phi             : Angle of Orientation Constraint of Parallel Leg Mechanism
                              (i.e., phi = q21 + q22 = q11 + q12 + q13) [units: radians] (would be always 0)
        """


        rot_wrist3 = util.quaternion_2_rotation_matrix(wrist_quaternion)
        T_shoulder_wrist3 = np.eye(4, dtype=np.float32)
        T_shoulder_wrist3[0:3,0:3] = rot_wrist3
        T_shoulder_wrist3[0:3,3] = np.array(shoulder_2_toe_xyz).reshape(-1)

        P_shoulder_wrist = T_shoulder_wrist3[0:3,3].reshape(-1)

        First_3_joint = Leg.leg_ik_direct_calculation_3DoF(P_shoulder_wrist, which_leg, is_first_ik, prev_angles)

        shoulder_angle = First_3_joint[0]
        q11 = First_3_joint[1]
        q21 = First_3_joint[2]

        if q11 < -np.pi/2:
            q11 = -np.pi-q11
        if q11 > np.pi/2:
            q11 = np.pi-q11
        if q21 < 0:
            q21 = -q21



        T_0_shoulder = Leg.leg_fk_direct_calculation(0.0, [shoulder_angle, q11, q21,0,0,0], 0, 1, use_quaternion = False)
        T_0_wrist = Leg.leg_fk_direct_calculation(0.0, [shoulder_angle, q11, q21,0,0,0], 0, 11, use_quaternion = False)
        T_shoulder_wrist = np.dot(np.linalg.inv(T_0_shoulder), T_0_wrist)

        T_wrist_wrist3 = np.dot(np.linalg.inv(T_shoulder_wrist), T_shoulder_wrist3)

        rot_wrist = T_wrist_wrist3[0:3,0:3]

        # Rotation Matrix of the wrist orientation with respect to the Wrist Reference Frame (before 3-DoF wrist
        # rotations). This can be thought of as the rotation matrix of the wrist (so we determine what the 3-DoF
        # spherical joint angle need to be).

        r_w_11, r_w_12, r_w_13, \
            r_w_21, r_w_22, r_w_23, \
            r_w_31, r_w_32, r_w_33 = util.unpack_rotation_matrix(rot_wrist)

        # Trivial solution to the Spherical Joint
        qw2 = np.arctan2(-r_w_23, np.sqrt(r_w_13 ** 2 + r_w_33 ** 2))
        qw1 = np.arctan2(- r_w_33 / np.cos(qw2),  r_w_13 / np.cos(qw2))
        qw3 = np.arctan2(- r_w_21 / np.cos(qw2), - r_w_22 / np.cos(qw2))

        return [shoulder_angle, q11, 0, 0, q21, 0, qw1, qw2, qw3, 0]


    @staticmethod
    def leg_ik_direct_calculation_3DoF(shoulder_2_toe_xyz, which_leg, is_first_ik = True, prev_angles=None):
        shoulder_angle = np.arctan2(shoulder_2_toe_xyz[1],shoulder_2_toe_xyz[0])
        if is_first_ik == False:
            prev_shoulder_angle = prev_angles[0]
            shoulder_angle2 = shoulder_angle
            if shoulder_angle<-np.pi/2:
                shoulder_angle2 = shoulder_angle + np.pi
            if shoulder_angle>np.pi/2:
                shoulder_angle2 = shoulder_angle - np.pi
            if abs(shoulder_angle2-prev_shoulder_angle) < abs(shoulder_angle-prev_shoulder_angle):
                shoulder_angle = shoulder_angle2

        T_shi_A_rot = np.array([[np.cos(shoulder_angle), -np.sin(shoulder_angle), 0, 0],
                               [np.sin(shoulder_angle), np.cos(shoulder_angle),  0, 0],
                               [                     0,                       0, 1, 0],
                               [0,0,0,1]])

        T_shi_A_t = np.array([[1, 0, 0, LEG_ORIGIN_X],
                              [0, 1, 0, 0],
                              [0, 0, 1, TOP_LEG_SERVO_OFF_Z],
                              [0,0,0,1]])

        T_shi_A = np.dot(T_shi_A_rot, T_shi_A_t)


        T_shi_A_cord_rot_1 = np.array([[1,                 0,                 0, 0],
                                       [0,   np.cos(np.pi/2),  -np.sin(np.pi/2), 0],
                                       [0,   np.sin(np.pi/2),   np.cos(np.pi/2), 0],
                                       [0,0,0,1]])

        T_shi_A_cord_rot_2 = np.array([[np.cos(-np.pi/2),  -np.sin(-np.pi/2),  0, 0],
                                       [np.sin(-np.pi/2),   np.cos(-np.pi/2),  0, 0],
                                       [               0,                  0,  1, 0],
                                       [0,0,0,1]])
        T_shi_A_cord_rot = np.dot(T_shi_A_cord_rot_1, T_shi_A_cord_rot_2)


        T_shi_A = np.dot(T_shi_A, T_shi_A_cord_rot)

        P_A = np.dot(np.linalg.inv(T_shi_A),np.array([[shoulder_2_toe_xyz[0]],[shoulder_2_toe_xyz[1]],[shoulder_2_toe_xyz[2]],[1]]))
        if is_first_ik == True:
            sol = Leg.leg_ik_2DoF(P_A[1, 0], P_A[0, 0])
        else:
            prev_q11_angle = prev_angles[1]
            prev_q21_angle = prev_angles[2]
            prev_q11q21 = np.array([prev_q11_angle, prev_q21_angle])

            min_dist = 2147483647.0
            dcase = 0
            for i in range(4):
                current_sol = Leg.leg_ik_2DoF(P_A[1, 0], P_A[0, 0], i, init_guess = prev_q11q21)
                current_sol = np.array(wrap_to_pi(np.array([current_sol[0], current_sol[1]]))).reshape(-1)
                dist = np.linalg.norm(current_sol-prev_q11q21)

                if dist<min_dist:
                    min_dist = dist
                    sol = current_sol
                    dcase = i




        sol = np.array([shoulder_angle, sol[0], sol[1]])
        sol = wrap_to_pi(sol)

        return sol

    @staticmethod
    def leg_ik_2DoF(y_ee, x_ee, case=0, init_guess = None):
        #case 0 : q11 -90 to 90 degrees / q21 0 to 180 degrees
        #case 1 : q11 90 to 180 degrees and -90 to -180 degrees / q21 0 to 180 degrees
        #case 2 : q11 -90 to 90 degrees / q21 0 to -180 degrees
        #case 3:  q11 90 to 180 degrees and -90 to -180 degrees / q21 0 to -180 degrees

        def func(x):
            if case == 0:
                sinq11 = np.sin(x[0])
                cosq21 = np.cos(x[1])
                cosq11 = np.sqrt(1-sinq11**2)
                sinq21 = np.sqrt(1-cosq21**2)
            elif case == 1:
                sinq11 = np.sin(x[0])
                cosq21 = np.cos(x[1])
                cosq11 = -np.sqrt(1-sinq11**2)
                sinq21 = np.sqrt(1-cosq21**2)
            elif case == 2:
                sinq11 = np.sin(x[0])
                cosq21 = np.cos(x[1])
                cosq11 = np.sqrt(1-sinq11**2)
                sinq21 = -np.sqrt(1-cosq21**2)
            else:
                sinq11 = np.sin(x[0])
                cosq21 = np.cos(x[1])
                cosq11 = -np.sqrt(1-sinq11**2)
                sinq21 = -np.sqrt(1-cosq21**2)

            L_GB = L_LEG_LINK_1 * (cosq11 - cosq21) - L_BLSP
            L_EG = L_LEG_LINK_1 * (sinq21 - sinq11)
            L_EB = np.sqrt(L_GB**2 + L_EG**2)

            COS_GEB = L_EG/L_EB
            SIN_GEB = L_GB/L_EB    

            COS_BEC = (L_EB**2 + L_LEG_LINK_2_NEW**2 - L_LEG_LINK_2**2) / (2*L_EB*L_LEG_LINK_2_NEW)
            SIN_BEC = np.sqrt(1 - COS_BEC**2)
  

            COS_GET = (COS_GEB*COS_BEC - SIN_GEB*SIN_BEC)*np.cos(LEG_THETA_1_OFF_ANGLE) - (SIN_GEB*COS_BEC + COS_GEB*SIN_BEC)*np.sin(LEG_THETA_1_OFF_ANGLE)
            SIN_GET = (SIN_GEB*COS_BEC + COS_GEB*SIN_BEC)*np.cos(LEG_THETA_1_OFF_ANGLE) + (COS_GEB*COS_BEC - SIN_GEB*SIN_BEC)*np.sin(LEG_THETA_1_OFF_ANGLE)


            L_EI = L_LEG_LINK_A22_WRIST * COS_GET
            L_IT = L_LEG_LINK_A22_WRIST * SIN_GET


            fk_variable_2 = L_LEG_LINK_1 * sinq21 - L_EI
            fk_variable_3 = L_LEG_LINK_1 * cosq21 + L_BLSP + L_IT

            return [fk_variable_2-y_ee, fk_variable_3-x_ee]

        if init_guess is None:
            root = fsolve(func,[0.0,np.pi/2])
        else:
            root = fsolve(func, init_guess)
        root = wrap_to_pi(root)
        if case == 0:
            if root[0]>np.pi/2:
                root[0] = np.pi - root[0]
            if root[0]<-np.pi/2:
                root[0] = -np.pi - root[0]
            if root[1]<0:
                root[1] = -root[1]
        elif case == 1:
            if root[0]>=0 and root[0]<np.pi/2:
                root[0] = np.pi - root[0]
            if root[0]<0 and root[0]>-np.pi/2:
                root[0] = -np.pi - root[0]
            if root[1]<0:
                root[1] = -root[1]
        elif case == 2:
            if root[0]>np.pi/2:
                root[0] = np.pi - root[0]
            if root[0]<-np.pi/2:
                root[0] = -np.pi - root[0]
            if root[1]>0:
                root[1] = -root[1]
        else:
            if root[0]>=0 and root[0]<np.pi/2:
                root[0] = np.pi - root[0]
            if root[0]<0 and root[0]>-np.pi/2:
                root[0] = -np.pi - root[0]
            if root[1]>0:
                root[1] = -root[1]



        return root


    @staticmethod
    def leg_ik(shoulder_2_toe_xyz, wrist_quaternion, which_leg=-1):
        """ Calculate the Inverse Kinematics of the Parallel Leg.
        This method calculates the inverse kinematics of the parallel leg for the SCALER_v2 given the position of the
        toe (position of the toe or in our case, the center of the gripper) and the desired wrist orientation.
        Args:
            shoulder_2_toe_xyz: Vector in 3D-space (X, Y, Z) from the Shoulder Joint to the desired Toe Position.
                                [dim: 3 x 1][units: mm]
            wrist_quaternion: Quaternion (w, x, y, z) that represents the desired wrist orientation with respect to the
                              Body Frame of SCALER_v2. [dim: 1 x 4]
            which_leg (optional arg): This is an index variable that specifies which leg we want to find the shoulder
                                      vertex for. [default value: -1].
                For SCALER_v1, we have four legs in total. This means that which_leg has the following options
                which_leg = 0: Leg 1 (Front Right Leg).
                which_leg = 1: Leg 2 (Back Right Leg).
                which_leg = 2: Leg 3 (Back Left Leg).
                which_leg = 3: Leg 4 (Front Left Leg).
        Returns:
            shoulder_angle  : Angle of the Shoulder Joint [units: radians]
            q11             : Angle of the Top Leg Servo Joint [units: radians]
            q21             : Angle of the Bottom Leg Servo Joint [units: radians]
            qw1             : Angle of the First Wrist Servo Joint [units: radians]
            qw2             : Angle of the Second Wrist Servo Joint [units: radians]
            qw3             : Angle of the Third Wrist Servo Joint [units: radians]
        """
        if which_leg == -1:
            [shoulder_angle, q11, q12, q13, q21, q22, qw1, qw2, qw3, phi] = \
                Leg.leg_ik_direct_calculation(shoulder_2_toe_xyz, wrist_quaternion)
        else:
            [shoulder_angle, q11, q12, q13, q21, q22, qw1, qw2, qw3, phi] = \
                Leg.leg_ik_direct_calculation(shoulder_2_toe_xyz, wrist_quaternion, which_leg=which_leg)

        joint_angles = [shoulder_angle, q11, q21, qw1, qw2, qw3]

        return wrap_to_pi(joint_angles)

    @staticmethod
    def measurment_orientation_6DoF_gripper_new(toe1, toe2, body_angle, FT_ft_frame, F_finger1_frame, F_finger2_frame, whichLeg):
        return 0

    @staticmethod
    def measurment_orientation_6DoF_gripper(toe1, toe2, body_angle, FT_ft_frame, F_finger1_frame, F_finger2_frame, whichLeg):
        # OUTPUT: toe 1 and 2 position relative to gripper center frame, FT sensor value relative to gripper center frame,
        # and toe1 and 2 forces relative to gripper center frame


        # We calculate the linear actuator and gripper offset angle value (i.e., IK of gripper only)
        L_actuator, theta_actuator, T_Gripper_Center_Body = scaler_std_utils.gripper_2d_two_finger_ik(toe1,toe2)


        # Unpack the 2D Gripper Hardware Constants (Link Lengths)
        L_GRIPPER_2D_L_IJ_X = robot_consts.L_GRIPPER_2D_L_IJ_X
        L_GRIPPER_2D_L_IJ_Y = robot_consts.L_GRIPPER_2D_L_IJ_Y

        L_GRIPPER_2D_L1 = robot_consts.L_GRIPPER_2D_L1

        L_GRIPPER_2D_L2 = robot_consts.L_GRIPPER_2D_L2
        L_GRIPPER_2D_L3 = robot_consts.L_GRIPPER_2D_L3
        L_GRIPPER_2D_L4 = robot_consts.L_GRIPPER_2D_L4
        L_GRIPPER_2D_L5 = robot_consts.L_GRIPPER_2D_L5
        L_GRIPPER_2D_L6 = robot_consts.L_GRIPPER_2D_L6
        L_GRIPPER_2D_L7 = robot_consts.L_GRIPPER_2D_L7

        L_GRIPPER_2D_L8 = robot_consts.L_GRIPPER_2D_L8
        L_GRIPPER_2D_L9 = robot_consts.L_GRIPPER_2D_L9
        L_GRIPPER_2D_L10 = robot_consts.L_GRIPPER_2D_L10
        L_GRIPPER_2D_L11 = robot_consts.L_GRIPPER_2D_L11
        L_GRIPPER_2D_L12 = robot_consts.L_GRIPPER_2D_L12
        L_GRIPPER_2D_L13 = robot_consts.L_GRIPPER_2D_L13

        L_GRIPPER_2D_L14 = robot_consts.L_GRIPPER_2D_L14
        L_GRIPPER_2D_L15 = robot_consts.L_GRIPPER_2D_L15

        x_D = L_actuator * np.cos(theta_actuator)
        y_D = L_actuator * np.sin(theta_actuator)

        # Calculating theta3, theta4, given x_D and y_D
        r_BD = np.sqrt((x_D - L_GRIPPER_2D_L1) ** 2 + (y_D - L_GRIPPER_2D_L2) ** 2)

        theta3 = np.arccos((r_BD ** 2 + L_GRIPPER_2D_L3 ** 2 - L_GRIPPER_2D_L4 ** 2) / (2 * r_BD * L_GRIPPER_2D_L3)) + \
                 np.arctan2(y_D - L_GRIPPER_2D_L2, x_D - L_GRIPPER_2D_L1)

        theta4 = theta3 + \
                 np.arccos((L_GRIPPER_2D_L3 ** 2 + L_GRIPPER_2D_L4 ** 2 - r_BD ** 2) /
                           (2 * L_GRIPPER_2D_L3 * L_GRIPPER_2D_L4)) - np.pi

        # Calculating theta6, theta5, given x_D and y_D
        r_FD = np.sqrt((x_D - L_GRIPPER_2D_L1) ** 2 + (y_D + L_GRIPPER_2D_L7) ** 2)

        theta6 = np.arctan2(y_D + L_GRIPPER_2D_L7, x_D - L_GRIPPER_2D_L1) - \
                 np.arccos((r_FD ** 2 + L_GRIPPER_2D_L6 ** 2 - L_GRIPPER_2D_L5 ** 2) / (2 * r_FD * L_GRIPPER_2D_L6))

        theta5 = np.pi - \
                 np.arccos((L_GRIPPER_2D_L6 ** 2 + L_GRIPPER_2D_L5 ** 2 - r_FD ** 2) /
                           (2 * L_GRIPPER_2D_L6 * L_GRIPPER_2D_L5)) + theta6

        # Calculating x_G, y_G, given theta3, and theta4 (Due to closed chain constraint)
        x_G = L_GRIPPER_2D_L1 + L_GRIPPER_2D_L3 * np.cos(theta3) + L_GRIPPER_2D_L13 * np.cos(theta4 - np.pi)
        y_G = L_GRIPPER_2D_L2 + L_GRIPPER_2D_L3 * np.sin(theta3) + L_GRIPPER_2D_L13 * np.sin(theta4 - np.pi)

        # Calculating theta11, theta12, given x_G and y_G
        r_IG = np.sqrt((x_G - L_GRIPPER_2D_L_IJ_X) ** 2 + (y_G - L_GRIPPER_2D_L_IJ_Y / 2) ** 2)

        theta11 = np.arccos((r_IG ** 2 + L_GRIPPER_2D_L11 ** 2 - L_GRIPPER_2D_L12 ** 2) /
                            (2 * r_IG * L_GRIPPER_2D_L11)) + \
                  np.arctan2(y_G - L_GRIPPER_2D_L_IJ_Y / 2, x_G - L_GRIPPER_2D_L_IJ_X)

        theta12 = theta11 + np.arccos((L_GRIPPER_2D_L11 ** 2 + L_GRIPPER_2D_L12 ** 2 - r_IG ** 2) /
                                      (2 * L_GRIPPER_2D_L11 * L_GRIPPER_2D_L12)) - np.pi

        # Calculating x_L, y_L, given theta6, and theta5 (Due to closed chain constraint)
        x_L = L_GRIPPER_2D_L1 + L_GRIPPER_2D_L6 * np.cos(theta6) + L_GRIPPER_2D_L10 * np.cos(theta5 - np.pi)
        y_L = -L_GRIPPER_2D_L7 + L_GRIPPER_2D_L6 * np.sin(theta6) + L_GRIPPER_2D_L10 * np.sin(theta5 - np.pi)

        # Calculating theta8, theta9, given x_L and y_L
        r_JK = np.sqrt((x_L - L_GRIPPER_2D_L_IJ_X) ** 2 + (y_L + L_GRIPPER_2D_L_IJ_Y / 2) ** 2)

        theta8 = np.arctan2(y_L + L_GRIPPER_2D_L_IJ_Y / 2, x_L - L_GRIPPER_2D_L_IJ_X) - \
                 np.arccos((r_JK ** 2 + L_GRIPPER_2D_L8 ** 2 - L_GRIPPER_2D_L9 ** 2) / (2 * r_JK * L_GRIPPER_2D_L8))

        theta9 = np.pi - \
                 np.arccos((L_GRIPPER_2D_L8 ** 2 + L_GRIPPER_2D_L9 ** 2 - r_JK ** 2) /
                           (2 * L_GRIPPER_2D_L8 * L_GRIPPER_2D_L9)) + theta8

        # Assign equivalent theta angles
        theta10 = theta5
        theta13 = theta4
        theta14 = theta12
        theta15 = theta9

        # Position of Point M (One of the Finger Tips)
        x_M = L_GRIPPER_2D_L_IJ_X + L_GRIPPER_2D_L11 * np.cos(theta11) + \
              (L_GRIPPER_2D_L12 + L_GRIPPER_2D_L14) * np.cos(theta12)

        y_M = L_GRIPPER_2D_L_IJ_Y / 2 + L_GRIPPER_2D_L11 * np.sin(theta11) + \
              (L_GRIPPER_2D_L12 + L_GRIPPER_2D_L14) * np.sin(theta12)

        # Position of Point N (the other Finger Tip)
        x_N = L_GRIPPER_2D_L_IJ_X + L_GRIPPER_2D_L8 * np.cos(theta8) + \
              (L_GRIPPER_2D_L9 + L_GRIPPER_2D_L15) * np.cos(theta9)

        y_N = - L_GRIPPER_2D_L_IJ_Y / 2 + L_GRIPPER_2D_L8 * np.sin(theta8) + \
              (L_GRIPPER_2D_L9 + L_GRIPPER_2D_L15) * np.sin(theta9)

        T_gripper_origin_toe_M = np.array([[np.cos(theta12), -np.sin(theta12), 0, x_M],
                                           [np.sin(theta12), np.cos(theta12), 0, y_M],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]])

        T_gripper_origin_toe_N = np.array([[np.cos(theta9), -np.sin(theta9), 0, x_N],
                                           [np.sin(theta9), np.cos(theta9), 0, y_N],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]])

        T_gripper_center_gripper_origin = np.array([[0.0, 0.0, -1.0, 0.0],
                                                    [0.0, 1.0, 0.0, 0.0],
                                                    [1.0, 0.0, 0.0, -L_GRIPPER_2D_L1],
                                                    [0.0, 0.0, 0.0, 1.0]])

        T_gripper_center_toe_M = np.matmul(T_gripper_center_gripper_origin, T_gripper_origin_toe_M)
        T_gripper_center_toe_N = np.matmul(T_gripper_center_gripper_origin, T_gripper_origin_toe_N)

        # TODO: NO Z VALUES FOR FINGERTIP POSITION RELATIVE TO CENTER? DOES THAT MAKE SENSE?
        p1 = T_gripper_center_toe_M[0:3,3]
        p2 = T_gripper_center_toe_N[0:3,3]


        shoulder_vertex = Scaler_utils.ScalerStandardUtilMethods.find_shoulder_vertices(body_angle,
                                                                                                use_find_specific_shoulder=True,
                                                                                                which_leg=whichLeg)
        shoulder_2_toe_xyz = T_Gripper_Center_Body[0:3,3]-shoulder_vertex
        wrist_quaternion = util.rotation_matrix_2_quaternion(T_Gripper_Center_Body[0:3,0:3])


        rot_gripper_center_body = T_Gripper_Center_Body[0:3,0:3]


        shoulder_angle, px_wrist_leg, py_wrist_leg, pz_wrist_leg = \
            Scaler_utils.ScalerStandardUtilMethods.find_position_wrist_2_leg_6dof(shoulder_2_toe_xyz,
                                                                                  wrist_quaternion, L_WRIST,
                                                                                  L_GRIPPER_OFFSET_X,
                                                                                  L_GRIPPER_OFFSET_Y,
                                                                                  LEG_ORIGIN_X, LEG_ORIGIN_Z,
                                                                                  LEG_ROT_OFF_ANGLE,
                                                                                  which_leg=whichLeg)

        q11, q12, q13, q21, q22, phi = Scaler_utils.SCALERv2UtilMethods.leg_v2_ik_direct(px_wrist_leg, py_wrist_leg,
                                                                                         L_LEG_LINK_1, L_LEG_LINK_2,
                                                                                         L_LEG_LINK_A23_WRIST, L_BLSP,
                                                                                         LEG_GAMMA_OFF_ANGLE,
                                                                                         LEG_THETA_1_OFF_ANGLE)

        # Rotation matrix of the Wrist Reference Frame (before 3-DoF wrist rotations) with respect to SCALER v2 Body
        # Reference Frame.
        rot_wrist_body = np.array(
            [[np.cos(shoulder_angle) * np.sin(phi + LEG_THETA_1_OFF_ANGLE),
              np.cos(shoulder_angle) * np.cos(phi + LEG_THETA_1_OFF_ANGLE),
              np.sin(shoulder_angle)],
             [np.sin(shoulder_angle) * np.sin(phi + LEG_THETA_1_OFF_ANGLE),
              np.sin(shoulder_angle) * np.cos(phi + LEG_THETA_1_OFF_ANGLE),
              -np.cos(shoulder_angle)],
             [-np.cos(phi + LEG_THETA_1_OFF_ANGLE),
              np.sin(phi + LEG_THETA_1_OFF_ANGLE),
              0.0]])

        # Rotation matrix of the wrist frame to the force / torque sensor frame is a -90 degree rotation along y-axis
        rot_FT_wrist = np.transpose(np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])).reshape(3, 3)

        rot_FT_body = np.matmul(rot_wrist_body, rot_FT_wrist)

        rot_FT_gripper_center = np.matmul(np.linalg.inv(rot_gripper_center_body), rot_FT_body)

        # Get force and torque in gripper frame
        Force_values = FT_ft_frame[0:3].reshape(3, 1)
        Torque_values = FT_ft_frame[3:6].reshape(3, 1)

        Force_gripper_center_frame = np.matmul(rot_FT_gripper_center, Force_values)
        Torque_gripper_center_frame = np.matmul(rot_FT_gripper_center, Torque_values)
        FT_gripper_center_frame = np.vstack([Force_gripper_center_frame, Torque_gripper_center_frame])

        # now we calculate the force values of fingertip in the gripper frame
        rot_finger1_gripper_center = T_gripper_center_toe_M[0:3, 0:3]
        rot_finger2_gripper_center = T_gripper_center_toe_N[0:3, 0:3]

        #euler_angle = util.rotation_2_euler(rot_finger1_gripper_center)
        #euler_angle[1] = 0
        #rot_finger1_gripper_center = util.euler_2_rotation(euler_angle)

        F_finger1_gripper_center_frame = np.matmul(rot_finger1_gripper_center, F_finger1_frame)
        F_finger2_gripper_center_frame = np.matmul(rot_finger2_gripper_center, F_finger2_frame)


        return p1, p2, F_finger1_gripper_center_frame, F_finger2_gripper_center_frame, FT_gripper_center_frame, rot_gripper_center_body


    @staticmethod
    def FT_orientation_calculation_6DoF_gripper_old(shoulder_2_toe_xyz, wrist_quaternion, toe1, toe2, FT_ft_frame, F_finger1_frame, F_finger2_frame, whichLeg):
        """ Calculates the Inverse Kinematics of the Parallel Leg and Wrist.
                This method calculates the inverse kinematics of the parallel leg for the SCALER_v2 given the position of
                the toe and the desired orientation of the spherical wrist (quaternion).
                Args:
                    shoulder_2_toe_xyz: Vector in 3D-space (X, Y, Z) from the Shoulder Joint to the desired Toe Position.
                                        [dim: 3 x 1][units: mm]
                    wrist_quaternion: Quaternion (w, x, y, z) that represents the desired wrist orientation with respect to the
                                      Body Frame of SCALER_v2. [dim: 1 x 4]
                    which_leg (optional arg): This is an index variable that specifies which leg we want to find the shoulder
                                              vertex for. [default value: -1].
                        For SCALER_v1, we have four legs in total. This means that which_leg has the following options
                        which_leg = 0: Leg 1 (Front Right Leg).
                        which_leg = 1: Leg 2 (Back Right Leg).
                        which_leg = 2: Leg 3 (Back Left Leg).
                        which_leg = 3: Leg 4 (Front Left Leg).
                Returns:
                    shoulder_angle  : Angle of the Shoulder Joint [units: radians]
                    q11             : Angle of the Top Leg Servo Joint [units: radians]
                    q12             : Angle of the Top Leg Elbow Joint [units: radians]
                    q13             : Angle to satisfy five-bar closed chain (q13 = q21 + q22 - q11 - q12) [units: radians]
                    q21             : Angle of the Bottom Leg Servo Joint [units: radians]
                    q22             : Angle of the Bottom Leg Elbow Joint [units: radians]
                    qw1             : Angle of the First Wrist Servo Joint [units: radians]
                    qw2             : Angle of the Second Wrist Servo Joint [units: radians]
                    qw3             : Angle of the Third Wrist Servo Joint [units: radians]
                    phi             : Angle of Orientation Constraint of Parallel Leg Mechanism
                                      (i.e., phi = q21 + q22 = q11 + q12 + q13) [units: radians]
                """
        shoulder_angle, px_wrist_leg, py_wrist_leg, pz_wrist_leg = \
            Scaler_utils.ScalerStandardUtilMethods.find_position_wrist_2_leg_6dof(shoulder_2_toe_xyz,
                                                                                  wrist_quaternion, L_WRIST,
                                                                                  L_GRIPPER_OFFSET_X,
                                                                                  L_GRIPPER_OFFSET_Y,
                                                                                  LEG_ORIGIN_X, LEG_ORIGIN_Z,
                                                                                  LEG_ROT_OFF_ANGLE, whichLeg)

        q11, q12, q13, q21, q22, phi = Scaler_utils.SCALERv2UtilMethods.leg_v2_ik_direct(px_wrist_leg, py_wrist_leg,
                                                                                         L_LEG_LINK_1, L_LEG_LINK_2,
                                                                                         L_LEG_LINK_A23_WRIST, L_BLSP,
                                                                                         LEG_GAMMA_OFF_ANGLE,
                                                                                         LEG_THETA_1_OFF_ANGLE)

        # Rotation matrix of the Wrist Reference Frame (before 3-DoF wrist rotations) with respect to SCALER v2 Body
        # Reference Frame.
        rot_b_wrist_frame = np.array(
            [[np.cos(shoulder_angle) * np.sin(phi + LEG_THETA_1_OFF_ANGLE),
              np.cos(shoulder_angle) * np.cos(phi + LEG_THETA_1_OFF_ANGLE),
              np.sin(shoulder_angle)],
             [np.sin(shoulder_angle) * np.sin(phi + LEG_THETA_1_OFF_ANGLE),
              np.sin(shoulder_angle) * np.cos(phi + LEG_THETA_1_OFF_ANGLE),
              -np.cos(shoulder_angle)],
             [-np.cos(phi + LEG_THETA_1_OFF_ANGLE),
              np.sin(phi + LEG_THETA_1_OFF_ANGLE),
              0.0]])

        # Rotation Matrix of the desired wrist orientation with respect to SCALER v2 Body Reference Frame.
        rot_desired_wrist = util.quaternion_2_rotation_matrix(wrist_quaternion)

        # Rotation Matrix of the wrist orientation with respect to the Wrist Reference Frame (before 3-DoF wrist
        # rotations). This can be thought of as the rotation matrix of the wrist (so we determine what the 3-DoF
        # spherical joint angle need to be).
        rot_wrist_base = np.dot(np.linalg.inv(rot_b_wrist_frame), rot_desired_wrist)

        r_w_11, r_w_12, r_w_13, \
        r_w_21, r_w_22, r_w_23, \
        r_w_31, r_w_32, r_w_33 = util.unpack_rotation_matrix(rot_wrist_base)

        # Trivial solution to the Spherical Joint
        qw2 = np.arctan2(-r_w_23, np.sqrt(r_w_13 ** 2 + r_w_33 ** 2))
        qw1 = np.arctan2(- r_w_33 / np.cos(qw2), r_w_13 / np.cos(qw2))
        qw3 = np.arctan2(- r_w_21 / np.cos(qw2), - r_w_22 / np.cos(qw2))

        fk_variable_0, fk_variable_1, fk_variable_2, fk_variable_3, fk_variable_4, fk_variable_5, fk_variable_6 = \
            Leg.find_fk_variables(q11, q21, qw1, qw2, qw3, whichLeg)

        phi_v2 = np.arctan2(fk_variable_2 - L_LEG_LINK_1 * np.sin(q21),
                            fk_variable_3 - L_LEG_LINK_1 * np.cos(q21) - L_BLSP)

        r_11 = np.cos(qw1) * np.cos(qw3) * np.sin(shoulder_angle) - \
               np.cos(phi_v2) * np.cos(shoulder_angle) * np.cos(qw2) * np.sin(qw3) + \
               np.cos(shoulder_angle) * np.cos(qw3) * np.sin(phi_v2) * np.sin(qw1) + \
               np.sin(shoulder_angle) * np.sin(qw1) * np.sin(qw2) * np.sin(qw3) - \
               np.cos(shoulder_angle) * np.cos(qw1) * np.sin(phi_v2) * np.sin(qw2) * np.sin(qw3)

        r_12 = np.cos(qw3) * np.sin(shoulder_angle) * np.sin(qw1) * np.sin(qw2) - \
               np.cos(phi_v2) * np.cos(shoulder_angle) * np.cos(qw2) * np.cos(qw3) - \
               np.cos(shoulder_angle) * np.sin(phi_v2) * np.sin(qw1) * np.sin(qw3) - \
               np.cos(qw1) * np.sin(shoulder_angle) * np.sin(qw3) - \
               np.cos(shoulder_angle) * np.cos(qw1) * np.cos(qw3) * np.sin(phi_v2) * np.sin(qw2)

        r_13 = np.cos(shoulder_angle) * np.cos(qw1) * np.cos(qw2) * np.sin(phi_v2) - \
               np.cos(phi_v2) * np.cos(shoulder_angle) * np.sin(qw2) - \
               np.cos(qw2) * np.sin(shoulder_angle) * np.sin(qw1)

        r_21 = np.cos(qw3) * np.sin(phi_v2) * np.sin(shoulder_angle) * np.sin(qw1) - \
               np.cos(phi_v2) * np.cos(qw2) * np.sin(shoulder_angle) * np.sin(qw3) - \
               np.cos(shoulder_angle) * np.cos(qw1) * np.cos(qw3) - \
               np.cos(shoulder_angle) * np.sin(qw1) * np.sin(qw2) * np.sin(qw3) - \
               np.cos(qw1) * np.sin(phi_v2) * np.sin(shoulder_angle) * np.sin(qw2) * np.sin(qw3)

        r_22 = np.cos(shoulder_angle) * np.cos(qw1) * np.sin(qw3) - \
               np.cos(phi_v2) * np.cos(qw2) * np.cos(qw3) * np.sin(shoulder_angle) - \
               np.cos(shoulder_angle) * np.cos(qw3) * np.sin(qw1) * np.sin(qw2) - \
               np.sin(phi_v2) * np.sin(shoulder_angle) * np.sin(qw1) * np.sin(qw3) - \
               np.cos(qw1) * np.cos(qw3) * np.sin(phi_v2) * np.sin(shoulder_angle) * np.sin(qw2)

        r_23 = np.cos(shoulder_angle) * np.cos(qw2) * np.sin(qw1) - \
               np.cos(phi_v2) * np.sin(shoulder_angle) * np.sin(qw2) + \
               np.cos(qw1) * np.cos(qw2) * np.sin(phi_v2) * np.sin(shoulder_angle)

        r_31 = np.cos(phi_v2) * np.cos(qw1) * np.sin(qw2) * np.sin(qw3) - \
               np.cos(phi_v2) * np.cos(qw3) * np.sin(qw1) - \
               np.cos(qw2) * np.sin(phi_v2) * np.sin(qw3)

        r_32 = np.cos(phi_v2) * np.sin(qw1) * np.sin(qw3) - \
               np.cos(qw2) * np.cos(qw3) * np.sin(phi_v2) + \
               np.cos(phi_v2) * np.cos(qw1) * np.cos(qw3) * np.sin(qw2)

        r_33 = - np.sin(phi_v2) * np.sin(qw2) - \
               np.cos(phi_v2) * np.cos(qw1) * np.cos(qw2)

        # Rotation Matrix of the Spherical Wrist Joint
        rot_gripper_base = np.array([[r_11, r_12, r_13],
                                          [r_21, r_22, r_23],
                                          [r_31, r_32, r_33]])


        # Rotation matrix of the wrist frame to the force / torque sensor frame is a -90 degree rotation along y-axis
        rot_FT_wrist = np.transpose(np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])).reshape(3, 3)

        # Rotation matrix of wrist frame relative to gripper frame
        rot_FT_base = np.matmul(rot_wrist_base,rot_FT_wrist)

        # Rotation matrix of FT sensor frame relative to gripper frame
        rot_FT_gripper = np.matmul(np.linalg.inv(rot_gripper_base),rot_FT_base)

        # Get force and torque in gripper frame
        Force_values = FT_ft_frame[0:3].reshape(3, 1)
        Torque_values = FT_ft_frame[3:6].reshape(3, 1)
        Force_gripper_frame = np.matmul(rot_FT_gripper, Force_values)
        Torque_gripper_frame = np.matmul(rot_FT_gripper, Torque_values)
        FT_gripper_frame = np.vstack([Force_gripper_frame, Torque_gripper_frame])

        # now we calculate the force values of fingertip in the gripper frame
        rot_finger1_base = util.quaternion_2_rotation_matrix(toe1[3:].reshape(1, 4))
        rot_finger2_base = util.quaternion_2_rotation_matrix(toe2[3:].reshape(1, 4))

        rot_base_gripper = np.linalg.inv(rot_gripper_base)

        rot_finger1_gripper = np.matmul(rot_base_gripper, rot_finger1_base)
        rot_finger2_gripper = np.matmul(rot_base_gripper, rot_finger2_base)

        F_finger1_gripper_frame = np.matmul(rot_finger1_gripper,F_finger1_frame)
        F_finger2_gripper_frame = np.matmul(rot_finger2_gripper,F_finger2_frame)


        return FT_gripper_frame, F_finger1_gripper_frame, F_finger2_gripper_frame

    @staticmethod
    def get_leg_motor_number(motor_id):
        """
        Receive the ID of a motor, return the number of leg and the number of joint that the motor is located at
        Note this function is specific for 3DOF walking configuration with predefined motor IDs!
        :param motor_id: ID of a motor
        :return: whichLeg: The number of Leg for motor_id
                 whichMotor: The number of joint for motor_id
        """
        # TODO: Check to make sure this was done properly Xuan [from: Zach]

        if motor_id == 0:
            assert False, "From get_leg_motor_number: wrong motor ID - this is body motor!"

        if motor_id in [113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124]:
            assert False, "From get_leg_motor_number: wrist motors don't have a slave motor!"

        # Turn slave motor IDs into master IDs
        if motor_id > 100:
            motor_id -= 100

        if motor_id in [1, 2, 3, 13, 14, 15]:  # RF leg
            whichLeg = 0
            if motor_id == 13:
                whichMotor = 3
            elif motor_id == 14:
                whichMotor = 4
            elif motor_id == 15:
                whichMotor = 5
            else:
                whichMotor = motor_id - 1
            return whichLeg, whichMotor

        elif motor_id in [4, 5, 6, 16, 17, 18]:  # RB leg
            whichLeg = 1
            if motor_id == 16:
                whichMotor = 3
            elif motor_id == 17:
                whichMotor = 4
            elif motor_id == 18:
                whichMotor = 5
            else:
                whichMotor = motor_id - 4
            return whichLeg, whichMotor

        elif motor_id in [7, 8, 9, 19, 20, 21]:  # LB leg
            whichLeg = 2
            if motor_id == 19:
                whichMotor = 3
            elif motor_id == 20:
                whichMotor = 4
            elif motor_id == 21:
                whichMotor = 5
            else:
                whichMotor = motor_id - 7
            return whichLeg, whichMotor

        elif motor_id in [10, 11, 12, 22, 23, 24]:  # LF leg
            whichLeg = 3
            if motor_id == 22:
                whichMotor = 3
            elif motor_id == 23:
                whichMotor = 4
            elif motor_id == 24:
                whichMotor = 5
            else:
                whichMotor = motor_id - 10
            return whichLeg, whichMotor

        else:
            assert False, "From get_leg_motor_number: Unknown motor ID!"

