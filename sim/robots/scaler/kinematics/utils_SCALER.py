__author__ = "Feng Xu, Zachary Lacey"
__email__ = "xufengmax@g.ucla.edu, zlacey1234@gmail.com"
__copyright__ = "Copyright 2021 RoMeLa"
__date__ = "July 30, 2021"

__version__ = "0.2.0"
__status__ = "Prototype"


from sim.robots.scalear_leg.kinematics import util
from .hardware_constants import consts, list_name_robots
import numpy as np
from scipy.optimize import fsolve, root
from scipy.spatial.transform import Rotation as R


#data_name = glob.read_one_data(glob.mem_settings, 'robot_name')
#robot_name = list_name_robots[int(data_name)]
robot_name = 'SCALER_climbing_6DoF_gripper'

robot_consts = consts[robot_name]

# Length of the Battery Link [units: mm]
L_BATTERY = robot_consts.L_BATTERY

# Length of the Body Link [units: mm]
L_BL = robot_consts.L_BL

# Length of the Servo to Rigid Body Link [units: mm]
L_S2RBx = robot_consts.L_S2RBx
L_S2RBy = robot_consts.L_S2RBy


class ScalerStandardUtilMethods:
    """ ScalerStandardUtilMethods Class
    This Class contains standard Utility Methods for SCALER. These methods should be standardized methods that are used
    applicable to both the SCALER_v1 and SCALER_v2.
    """

    @staticmethod
    def find_shoulder_vertices(posture_angle, use_find_specific_shoulder=False, which_leg=-1):
        """ Find the Shoulder Vertices for SCALER_v1
        This method calculates ONLY the shoulder vertices using the Link Constants (from hardware_constants.py) and the
        posture angle.
        Args:
            posture_angle: Angle of the Body Posture Joint (Motor ID: 17) [units: radians]
            use_find_specific_shoulder (optional arg): This is a Boolean flag which the user can specify as True if they
                                                       want to return the shoulder vertex for a single leg rather than
                                                       for all four legs. [default value: False]
            which_leg (optional arg): This is an index variable that specifies which leg we want to find the shoulder
                                      vertex for. [default value: -1].
                For SCALER_v1, we have four legs in total. This means that which_leg has the following options
                which_leg = 0: Leg 1 (Front Right Leg).
                which_leg = 1: Leg 2 (Back Right Leg).
                which_leg = 2: Leg 3 (Back Left Leg).
                which_leg = 3: Leg 4 (Front Left Leg).
        Returns:
            shoulder_vertices: All of the shoulder vertices of the SCALER_v1. [dim: 3 x 4][units: mm]
                if use_find_specific_shoulder = True, then
                shoulder_vertices: Is the shoulder vertex of the specified leg of SCALER_v1. [dim: 3 x 1][units: mm]
        """

        # Right Front Shoulder Vertex
        shoulder_rf = [L_S2RBx + (L_BL * np.cos(posture_angle) / 2),
                       (L_BATTERY / 2) + L_S2RBy - (L_BL * np.sin(posture_angle) / 2),
                       0.0]

        # Right Back Shoulder Vertex
        shoulder_rb = [L_S2RBx + (L_BL * np.cos(posture_angle) / 2),
                       -(L_BATTERY / 2) - L_S2RBy - (L_BL * np.sin(posture_angle) / 2),
                       0.0]

        # Left Back Shoulder Vertex
        shoulder_lb = [-L_S2RBx - (L_BL * np.cos(posture_angle) / 2),
                       -(L_BATTERY / 2) - L_S2RBy + (L_BL * np.sin(posture_angle) / 2),
                       0.0]

        # Left Front Shoulder Vertex
        shoulder_lf = [-L_S2RBx - (L_BL * np.cos(posture_angle) / 2),
                       (L_BATTERY / 2) + L_S2RBy + (L_BL * np.sin(posture_angle) / 2),
                       0.0]

        if use_find_specific_shoulder:
            if which_leg == 0:
                shoulder_vertices = shoulder_rf
            elif which_leg == 1:
                shoulder_vertices = shoulder_rb
            elif which_leg == 2:
                shoulder_vertices = shoulder_lb
            elif which_leg == 3:
                shoulder_vertices = shoulder_lf
            else:
                shoulder_vertices = -1
                print('Error! User did not specify a proper which_leg index. ')
        else:

            shoulder_vertices = np.array([shoulder_rf,
                                          shoulder_rb,
                                          shoulder_lb,
                                          shoulder_lf])
        return shoulder_vertices

    @staticmethod
    def find_body_vertices(posture_angle):
        """ Find the Body Vertices for SCALER_v1
        This method calculates the body vertices using the Link Constants (from hardware_constants.py) and the posture
        angle.
        Args:
            posture_angle: Angle of the Body Posture Joint (Motor ID: 17) [units: radians]
        Returns:
            body_vertices: All of the vertices of the SCALER_v1's body structure. [dim: 14 x 3], [units: mm]
        """
        # Body Structure Vertices
        point1 = [0.0,
                  L_BATTERY / 2,
                  0.0]

        point2 = [(L_BL * np.cos(posture_angle) / 2),
                  (L_BATTERY / 2) + (L_BL * np.sin(posture_angle) / 2),
                  0.0]

        point3 = [L_S2RB + (L_BL * np.cos(posture_angle) / 2),
                  (L_BATTERY / 2) + (L_BL * np.sin(posture_angle) / 2),
                  0.0]

        point4 = [L_S2RB + (L_BL * np.cos(posture_angle) / 2),
                  (L_BATTERY / 2) + L_S2RB + (L_BL * np.sin(posture_angle) / 2),
                  0.0]

        point5 = [0.0,
                  -L_BATTERY / 2,
                  0.0]

        point6 = [(L_BL * np.cos(posture_angle) / 2),
                  -(L_BATTERY / 2) + (L_BL * np.sin(posture_angle) / 2),
                  0.0]

        point7 = [L_S2RB + (L_BL * np.cos(posture_angle) / 2),
                  -(L_BATTERY / 2) + (L_BL * np.sin(posture_angle) / 2),
                  0.0]

        point8 = [L_S2RB + (L_BL * np.cos(posture_angle) / 2),
                  -(L_BATTERY / 2) - L_S2RB + (L_BL * np.sin(posture_angle) / 2),
                  0.0]

        point9 = [-(L_BL * np.cos(posture_angle) / 2),
                  -(L_BATTERY / 2) - (L_BL * np.sin(posture_angle) / 2),
                  0.0]

        point10 = [-L_S2RB - (L_BL * np.cos(posture_angle) / 2),
                   -(L_BATTERY / 2) - (L_BL * np.sin(posture_angle) / 2),
                   0.0]

        point11 = [-L_S2RB - (L_BL * np.cos(posture_angle) / 2),
                   -(L_BATTERY / 2) - L_S2RB - (L_BL * np.sin(posture_angle) / 2),
                   0.0]

        point12 = [-(L_BL * np.cos(posture_angle) / 2),
                   (L_BATTERY / 2) - (L_BL * np.sin(posture_angle) / 2),
                   0.0]

        point13 = [-L_S2RB - (L_BL * np.cos(posture_angle) / 2),
                   (L_BATTERY / 2) - (L_BL * np.sin(posture_angle) / 2),
                   0.0]

        point14 = [-L_S2RB - (L_BL * np.cos(posture_angle) / 2),
                   (L_BATTERY / 2) + L_S2RB - (L_BL * np.sin(posture_angle) / 2),
                   0.0]

        body_vertices = np.array([point1, point2, point3, point4, point5, point6, point7,
                                  point8, point9, point10, point11, point12, point13, point14])

        return body_vertices

    @staticmethod
    def find_rotation_shoulder_2_leg_frame(shoulder_2_toe_xyz, leg_origin_x, leg_origin_z,
                                           leg_rot_off_angle):
        """ Find Rotation Shoulder 2 Leg Frame
        This method solves is used to solve the rotation from the shoulder reference frame to the leg reference frame.
        Args:
            shoulder_2_toe_xyz: Vector in 3D-space (X, Y, Z) from the Shoulder Joint to the desired Toe Position.
                                [dim: 3 x 1][units: mm]
            leg_origin_x: Center Point Between the Top Leg Servo and the Bottom Leg Servo. (These X and Z represent the
                          Position (X,Z) of the Parallel Leg Origin with respect to Shoulder's Reference Frame)
                          [units: mm]
            leg_origin_z: Center Point Between the Top Leg Servo and the Bottom Leg Servo. (These X and Z represent the
                          Position (X,Z) of the Parallel Leg Origin with respect to Shoulder's Reference Frame)
                          [units: mm]
            leg_rot_off_angle: Rotation Angle Offset of the Parallel Leg Structure (Based on the X and Z Offset
                               Positions of the Top and Bottom Leg Servos). [units: radians]
        Returns:
            shoulder_angle  : Angle of the Shoulder Joint [units: radians]
            leg_2_toe_xyz_b : Vector from the Leg Origin to the Toe in the (Body Frame) [dim: 3 x 1][units: mm]
            rot_1_leg       : Rotation from the rotated Shoulder Frame to the Leg Frame (Rotation Matrix) [dim: 3 x 3]
            rot_sh_1        : Rotation about the Shoulder Joint (Rotation Matrix) [dim: 3 x 3]
        """
        shoulder_angle = np.arctan2(shoulder_2_toe_xyz[1], shoulder_2_toe_xyz[0])

        # Vector from the Leg Origin to the Toe in the (Body Frame)
        leg_2_toe_xyz_b = shoulder_2_toe_xyz - np.array([(leg_origin_x * np.cos(shoulder_angle)),
                                                         (leg_origin_x * np.sin(shoulder_angle)),
                                                         leg_origin_z])

        # Rotation about the Shoulder Joint (Rotation Matrices)
        rot_sh_1 = np.array([[np.cos(shoulder_angle), -np.sin(shoulder_angle), 0.0],
                             [np.sin(shoulder_angle), np.cos(shoulder_angle), 0.0],
                             [0.0, 0.0, 1.0]])

        rot_1_2 = np.array([[np.cos(leg_rot_off_angle + (np.pi / 2)), 0.0, np.sin(leg_rot_off_angle + (np.pi / 2))],
                            [0.0, 1.0, 0.0],
                            [-np.sin(leg_rot_off_angle + (np.pi / 2)), 0.0, np.cos(leg_rot_off_angle + (np.pi / 2))]])

        rot_2_leg = np.array([[1.0, 0.0, 0.0],
                              [0.0, 0.0, -1.0],
                              [0.0, 1.0, 0.0]])

        rot_1_leg = np.dot(rot_1_2, rot_2_leg)

        return shoulder_angle, leg_2_toe_xyz_b, rot_1_leg, rot_sh_1

    @staticmethod
    def find_position_wrist_2_leg_walking_4dof(shoulder_2_toe_xyz, wrist_angle, l_wrist, leg_origin_x, leg_origin_z,
                                               leg_rot_off_angle):
        """ Find Position Wrist 2 Leg 4DoF
        This method solves the vector of the Wrist Joint with Respect to the Leg Reference Frame for SCALER v2 4-DoF
        Configuration. This is specifically used to represent the Wrist Joint in the Leg Reference Frame (making it a
        2D point in the Parallel Leg Mechanism 2D workspace). Given this, pz_wrist_leg should be approximately zero and
        we use px_wrist_leg and py_wrist_leg to ultimately solve for the Leg Joint Angles.
        Args:
            shoulder_2_toe_xyz: Vector in 3D-space (X, Y, Z) from the Shoulder Joint to the desired Toe Position.
                                [dim: 3 x 1][units: mm]
            wrist_angle: Desired Angle of the Wrist (with respect to the XY Plane of the Body Frame of the
                         SCALER_v1). [units: radians]
            l_wrist: # Length of the Wrist Link [units: mm]
            leg_origin_x: Center Point Between the Top Leg Servo and the Bottom Leg Servo. (These X and Z represent the
                          Position (X,Z) of the Parallel Leg Origin with respect to Shoulder's Reference Frame)
                          [units: mm]
            leg_origin_z: Center Point Between the Top Leg Servo and the Bottom Leg Servo. (These X and Z represent the
                          Position (X,Z) of the Parallel Leg Origin with respect to Shoulder's Reference Frame)
                          [units: mm]
            leg_rot_off_angle: Rotation Angle Offset of the Parallel Leg Structure (Based on the X and Z Offset
                               Positions of the Top and Bottom Leg Servos). [units: radians]
        Returns:
            shoulder_angle  : Angle of the Shoulder Joint [units: radians]
            px_wrist_leg    : X-component of the wrist joint (with respect to the Leg Reference Frame). This is
                              essentially the x-component of the wrist joint in 2D Leg Plane (where the wrist joint is
                              in the Parallel Leg Mechanism's 2D workspace). [units: mm]
            py_wrist_leg    : Y-component of the wrist joint (with respect to the Leg Reference Frame). This is
                              essentially the y-component of the wrist joint in 2D Leg Plane (where the wrist joint is
                              in the Parallel Leg Mechanism's 2D workspace). [units: mm]
            pz_wrist_leg    : Z-component of the wrist joint (with respect to the Leg Reference Frame). This is
                              essentially the z-component of the wrist joint in 2D Leg Plane (where the wrist joint is
                              in the Parallel Leg Mechanism's 2D workspace).
                              NOTE: pz_wrist_leg should be approximately zero (or a very small number close to zero
                              i.e., e-10). [units: mm]
        """
        shoulder_angle, leg_2_toe_xyz_b, rot_1_leg, rot_sh_1 = \
            ScalerStandardUtilMethods.find_rotation_shoulder_2_leg_frame(shoulder_2_toe_xyz, leg_origin_x,
                                                                         leg_origin_z, leg_rot_off_angle)

        # Vector from the Leg Origin to the Wrist Joint in the (Leg Frame)
        leg_2_wrist_xyz_leg = np.dot(np.transpose(rot_1_leg),
                                     (np.dot(np.transpose(rot_sh_1),
                                             leg_2_toe_xyz_b) - np.array([l_wrist * np.cos(wrist_angle),
                                                                          0.0,
                                                                          l_wrist * np.sin(wrist_angle)])))

        px_wrist_leg = leg_2_wrist_xyz_leg[0]
        py_wrist_leg = leg_2_wrist_xyz_leg[1]
        pz_wrist_leg = leg_2_wrist_xyz_leg[2]
        return shoulder_angle, px_wrist_leg, py_wrist_leg, pz_wrist_leg

    @staticmethod
    def find_position_wrist_2_leg_walking_3dof(shoulder_2_toe_xyz, leg_origin_x, leg_origin_z,
                                               leg_rot_off_angle):
        """ Find Position Wrist 2 Leg 3DoF
        This method solves the vector of the Wrist Joint with Respect to the Leg Reference Frame for SCALER v2 3-DoF
        Configuration. This is specifically used to represent the Wrist Joint in the Leg Reference Frame (making it a
        2D point in the Parallel Leg Mechanism 2D workspace). Given this, pz_wrist_leg should be approximately zero and
        we use px_wrist_leg and py_wrist_leg to ultimately solve for the Leg Joint Angles.
        Args:
            shoulder_2_toe_xyz: Vector in 3D-space (X, Y, Z) from the Shoulder Joint to the desired Toe Position.
                                [dim: 3 x 1][units: mm]
            leg_origin_x: Center Point Between the Top Leg Servo and the Bottom Leg Servo. (These X and Z represent the
                          Position (X,Z) of the Parallel Leg Origin with respect to Shoulder's Reference Frame)
                          [units: mm]
            leg_origin_z: Center Point Between the Top Leg Servo and the Bottom Leg Servo. (These X and Z represent the
                          Position (X,Z) of the Parallel Leg Origin with respect to Shoulder's Reference Frame)
                          [units: mm]
            leg_rot_off_angle: Rotation Angle Offset of the Parallel Leg Structure (Based on the X and Z Offset
                               Positions of the Top and Bottom Leg Servos). [units: radians]
        Returns:
            shoulder_angle  : Angle of the Shoulder Joint [units: radians]
            px_wrist_leg    : X-component of the wrist joint (with respect to the Leg Reference Frame). This is
                              essentially the x-component of the wrist joint in 2D Leg Plane (where the wrist joint is
                              in the Parallel Leg Mechanism's 2D workspace). [units: mm]
            py_wrist_leg    : Y-component of the wrist joint (with respect to the Leg Reference Frame). This is
                              essentially the y-component of the wrist joint in 2D Leg Plane (where the wrist joint is
                              in the Parallel Leg Mechanism's 2D workspace). [units: mm]
            pz_wrist_leg    : Z-component of the wrist joint (with respect to the Leg Reference Frame). This is
                              essentially the z-component of the wrist joint in 2D Leg Plane (where the wrist joint is
                              in the Parallel Leg Mechanism's 2D workspace).
                              NOTE: pz_wrist_leg should be approximately zero (or a very small number close to zero
                              i.e., e-10). [units: mm]
        """

        shoulder_angle, leg_2_toe_xyz_b, rot_1_leg, rot_sh_1 = \
            ScalerStandardUtilMethods.find_rotation_shoulder_2_leg_frame(shoulder_2_toe_xyz, leg_origin_x,
                                                                         leg_origin_z, leg_rot_off_angle)

        # Vector from the Leg Origin to the Wrist Joint in the (Leg Frame)
        leg_2_wrist_xyz_leg = np.dot(np.transpose(rot_1_leg),
                                     (np.dot(np.transpose(rot_sh_1),
                                             leg_2_toe_xyz_b)))

        px_wrist_leg = leg_2_wrist_xyz_leg[0]
        py_wrist_leg = leg_2_wrist_xyz_leg[1]
        pz_wrist_leg = leg_2_wrist_xyz_leg[2]
        return shoulder_angle, px_wrist_leg, py_wrist_leg, pz_wrist_leg

    @staticmethod
    def find_position_wrist_2_leg_6dof(shoulder_2_toe_xyz, wrist_quaternion, l_wrist, l_gripper_offset_x,
                                       l_gripper_offset_y, leg_origin_x, leg_origin_z, leg_rot_off_angle, which_leg=-1):
        """ Find Position Wrist 2 Leg 6DoF
        This method solves the vector of the Wrist Joint with Respect to the Leg Reference Frame for SCALER v2 6-DoF
        Configuration. This is specifically used to represent the Wrist Joint in the Leg Reference Frame (making it a
        2D point in the Parallel Leg Mechanism 2D workspace). Given this, pz_wrist_leg should be approximately zero and
        we use px_wrist_leg and py_wrist_leg to ultimately solve for the Leg Joint Angles.
        Args:
            shoulder_2_toe_xyz: Vector in 3D-space (X, Y, Z) from the Shoulder Joint to the desired Toe Position.
                                [dim: 3 x 1][units: mm]
            wrist_quaternion: Quaternion (w, x, y, z) that represents the desired wrist orientation with respect to the
                              Body Frame of SCALER_v2. [dim: 4 x 1]
            l_wrist: Length of the Wrist Link [units: mm]
            l_gripper_offset_x: Length of the Gripper offset (x-component) from the Spherical Joint [units: mm]
            l_gripper_offset_y: Length of the Gripper offset (y-component) from the Spherical Joint [units: mm]
            leg_origin_x: Center Point Between the Top Leg Servo and the Bottom Leg Servo. (These X and Z represent the
                          Position (X,Z) of the Parallel Leg Origin with respect to Shoulder's Reference Frame)
                          [units: mm]
            leg_origin_z: Center Point Between the Top Leg Servo and the Bottom Leg Servo. (These X and Z represent the
                          Position (X,Z) of the Parallel Leg Origin with respect to Shoulder's Reference Frame)
                          [units: mm]
            leg_rot_off_angle: Rotation Angle Offset of the Parallel Leg Structure (Based on the X and Z Offset
                               Positions of the Top and Bottom Leg Servos). [units: radians]
            which_leg (optional arg): This is an index variable that specifies which leg we want to find the shoulder
                                      vertex for. [default value: -1].
                For SCALER_v1, we have four legs in total. This means that which_leg has the following options
                which_leg = 0: Leg 1 (Front Right Leg).
                which_leg = 1: Leg 2 (Back Right Leg).
                which_leg = 2: Leg 3 (Back Left Leg).
                which_leg = 3: Leg 4 (Front Left Leg).
        Returns:
            shoulder_angle  : Angle of the Shoulder Joint [units: radians]
            px_wrist_leg    : X-component of the wrist joint (with respect to the Leg Reference Frame). This is
                              essentially the x-component of the wrist joint in 2D Leg Plane (where the wrist joint is
                              in the Parallel Leg Mechanism's 2D workspace). [units: mm]
            py_wrist_leg    : Y-component of the wrist joint (with respect to the Leg Reference Frame). This is
                              essentially the y-component of the wrist joint in 2D Leg Plane (where the wrist joint is
                              in the Parallel Leg Mechanism's 2D workspace). [units: mm]
            pz_wrist_leg    : Z-component of the wrist joint (with respect to the Leg Reference Frame). This is
                              essentially the z-component of the wrist joint in 2D Leg Plane (where the wrist joint is
                              in the Parallel Leg Mechanism's 2D workspace).
                              NOTE: pz_wrist_leg should be approximately zero (or a very small number close to zero
                              i.e., e-10). [units: mm]
        """
        if which_leg == 0 or which_leg == 2:
            wrist_p = np.array([l_gripper_offset_x, l_gripper_offset_y, l_wrist])
        elif which_leg == 1 or which_leg == 3:
            wrist_p = np.array([l_gripper_offset_x, -l_gripper_offset_y, l_wrist])
        else:
            wrist_p = np.array([0, 0, 0])
            print('Error! User did not specify a proper which_leg index!!!')

        rot_body_wrist = util.quaternion_2_rotation_matrix(wrist_quaternion)

        p = np.dot(rot_body_wrist, wrist_p)

        shoulder_2_wrist_xyz = shoulder_2_toe_xyz - p

        shoulder_angle, px_wrist_leg, py_wrist_leg, pz_wrist_leg = \
            ScalerStandardUtilMethods.find_position_wrist_2_leg_walking_3dof(shoulder_2_wrist_xyz, leg_origin_x,
                                                                             leg_origin_z, leg_rot_off_angle)

        return shoulder_angle, px_wrist_leg, py_wrist_leg, pz_wrist_leg

    @staticmethod
    def gripper_kinematic_constraint(theta, *args):
        """ Defining the Gripper Kinematic Constraints
        This method defines the kinematic constraint of the gripper which is used to determine the gripper reference
        frame in the Gripper inverse kinematics.
        Args:
            theta: These are the angles that we aim to solve in the nonlinear solver.
                theta = [angle_BCD, angle_DEF, angle_GHI, angle_JKL]
            args:
        Returns:
            BI_IJ[0], BI_IJ[1], BI_IJ[2]: Cross Product of the r_BI and r_IJ vectors are equal to zero vector
                                          (equivalent to defining the constraint that r_BI and r_IJ are parallel)
            BI_IF[0], BI_IF[1], BI_IF[2]: Cross Product of the r_BI and r_IF vectors are equal to zero vector
                                          (equivalent to defining the constraint that r_BI and r_IF are parallel)
            BJ_JF[0], BJ_JF[1], BJ_JF[2]: Cross Product of the r_BJ and r_JF vectors are equal to zero vector
                                          (equivalent to defining the constraint that r_BJ and r_JF are parallel)
            IJ_JF[0], IJ_JF[1], IJ_JF[2]: Cross Product of the r_IJ and r_JF vectors are equal to zero vector
                                          (equivalent to defining the constraint that r_IJ and r_JF are parallel)
            r_IJ_mag - L_GRIPPER_2D_L_IJ_Y: Scalar value is equal to zero (equivalent to r_IJ_mag = L_GRIPPER_2DL_IJ_Y)
            r_BI_mag - (L_GRIPPER_2D_L2 - L_GRIPPER_2D_L_IJ_Y / 2): Scalar value is equal to zero (equivalent to
                                                                    r_BI_mag = (L_GRIPPER_2D_L2 -
                                                                                L_GRIPPER_2D_L_IJ_Y / 2)
            r_IF_mag - (L_GRIPPER_2D_L7 + L_GRIPPER_2D_L_IJ_Y / 2): Scalar value is equal to zero (equivalent to
                                                                    r_IF_mag - (L_GRIPPER_2D_L7 +
                                                                                L_GRIPPER_2D_L_IJ_Y / 2)
            r_BJ_mag - (L_GRIPPER_2D_L2 + L_GRIPPER_2D_L_IJ_Y / 2): Scalar value is equal to zero (equivalent to
                                                                    r_BJ_mag - (L_GRIPPER_2D_L2 +
                                                                                L_GRIPPER_2D_L_IJ_Y / 2)
            r_JF_mag - (L_GRIPPER_2D_L7 - L_GRIPPER_2D_L_IJ_Y / 2): Scalar value is equal to zero (equivalent to
                                                                    r_JF_mag - (L_GRIPPER_2D_L7 -
                                                                                L_GRIPPER_2D_L_IJ_Y / 2)
        """
        # Unpack the arguments
        position_C_body = args[0]
        body_R_C = args[1]
        position_E_body = args[2]
        body_R_E = args[3]
        position_H_body = args[4]
        body_R_H = args[5]
        position_K_body = args[6]
        body_R_K = args[7]

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

        # Calculate Position Gripper Point B (Body Frame)
        R_C_B = np.array([[np.cos(theta[0]), np.sin(theta[0]), 0],
                          [-np.sin(theta[0]), np.cos(theta[0]), 0],
                          [0, 0, 1]])

        position_B_body = position_C_body - np.dot(np.dot(body_R_C, R_C_B), np.array([L_GRIPPER_2D_L3, 0, 0]))

        # Calculate Position Gripper Point F (Body Frame)
        R_E_F = np.array([[np.cos(theta[1]), -np.sin(theta[1]), 0],
                          [np.sin(theta[1]), np.cos(theta[1]), 0],
                          [0, 0, 1]])

        position_F_body = position_E_body - np.dot(np.dot(body_R_E, R_E_F), np.array([L_GRIPPER_2D_L6, 0, 0]))

        # Calculate Position Gripper Point I (Body Frame)
        R_H_I = np.array([[np.cos(theta[2]), np.sin(theta[2]), 0],
                          [-np.sin(theta[2]), np.cos(theta[2]), 0],
                          [0, 0, 1]])

        position_I_body = position_H_body + np.dot(np.dot(body_R_H, R_H_I), np.array([L_GRIPPER_2D_L11, 0, 0]))

        # Calculate Position Gripper Point J (Body Frame)
        R_K_J = np.array([[np.cos(theta[3]), -np.sin(theta[3]), 0],
                          [np.sin(theta[3]), np.cos(theta[3]), 0],
                          [0, 0, 1]])

        position_J_body = position_K_body + np.dot(np.dot(body_R_K, R_K_J), np.array([L_GRIPPER_2D_L8, 0, 0]))

        # Vectors that represent the rigid structure that Points B, F, I and J of the Gripper lie on (these points lie
        # on a line)
        r_BI = position_I_body - position_B_body
        r_IJ = position_J_body - position_I_body
        r_IF = position_F_body - position_I_body
        r_BJ = position_J_body - position_B_body
        r_JF = position_F_body - position_J_body

        # Cross Product of different vectors. (cross product of all of these will be equal to zero since these vectors
        # are all parallel)
        BI_IJ = np.cross(r_BI, r_IJ)
        BI_IF = np.cross(r_BI, r_IF)

        BJ_JF = np.cross(r_BJ, r_JF)
        IJ_JF = np.cross(r_IJ, r_JF)

        # Magnitude of these vectors are also constrained to capture the fact that these vectors lie on the same line.
        r_IJ_mag = np.linalg.norm(r_IJ)
        r_BI_mag = np.linalg.norm(r_BI)
        r_IF_mag = np.linalg.norm(r_IF)
        r_BJ_mag = np.linalg.norm(r_BJ)
        r_JF_mag = np.linalg.norm(r_JF)

        return [BI_IJ[0], BI_IJ[1], BI_IJ[2],
                BI_IF[0], BI_IF[1], BI_IF[2],
                BJ_JF[0], BJ_JF[1], BJ_JF[2],
                IJ_JF[0], IJ_JF[1], IJ_JF[2],
                r_IJ_mag - L_GRIPPER_2D_L_IJ_Y,
                r_BI_mag - (L_GRIPPER_2D_L2 - L_GRIPPER_2D_L_IJ_Y / 2),
                r_IF_mag - (L_GRIPPER_2D_L7 + L_GRIPPER_2D_L_IJ_Y / 2),
                r_BJ_mag - (L_GRIPPER_2D_L2 + L_GRIPPER_2D_L_IJ_Y / 2),
                r_JF_mag - (L_GRIPPER_2D_L7 - L_GRIPPER_2D_L_IJ_Y / 2)]

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


        T_gripper_center_gripper_origin = np.array([[0.0,   0.0,    -1.0,   0.0],
                                                        [0.0,   1.0,    0.0,    0.0],
                                                        [1.0,   0.0,    0.0,    -L_GRIPPER_2D_L1],
                                                        [0.0,   0.0,    0.0,    1.0]])


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
                                           [np.sin(theta12),  np.cos(theta12), 0, y_M],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]])

        T_gripper_origin_toe_N = np.array([[np.cos(theta9), -np.sin(theta9), 0, x_N],
                                           [np.sin(theta9),  np.cos(theta9), 0, y_N],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]])

        T_shoulder_toe_M = np.dot(T_shoulder_gripper_origin, T_gripper_origin_toe_M)
        T_shoulder_toe_N = np.dot(T_shoulder_gripper_origin, T_gripper_origin_toe_N)

        R_M = T_shoulder_toe_M[0:3,0:3]
        R_M_quat = util.rotation_matrix_2_quaternion(R_M)
        P_M = T_shoulder_toe_M[:,3]
        R_N = T_shoulder_toe_N[0:3,0:3]
        R_N_quat = util.rotation_matrix_2_quaternion(R_N)
        P_N = T_shoulder_toe_N[:,3]

        fingertip1 = np.array([P_M[0],P_M[1],P_M[2],R_M_quat[0],R_M_quat[1],R_M_quat[2],R_M_quat[3]])
        fingertip2 = np.array([P_N[0],P_N[1],P_N[2],R_N_quat[0],R_N_quat[1],R_N_quat[2],R_N_quat[3]])

        return fingertip1.reshape(7,1), fingertip2.reshape(7,1)

    @staticmethod
    def gripper_2d_two_finger_fk_noOffset(T_shoulder_gripper_center, L_actuator, theta_actuator, which_leg):
        """
        Basically same to gripper_2d_two_finger_fk. The difference here is that it makes orientation of finger local frames
        be SAME to wrist orientation frame
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

        fingertip1 = np.array([P_M[0], P_M[1], P_M[2], R_M_quat[0], R_M_quat[1], R_M_quat[2], R_M_quat[3]]).reshape(7,1)
        fingertip2 = np.array([P_N[0], P_N[1], P_N[2], R_N_quat[0], R_N_quat[1], R_N_quat[2], R_N_quat[3]]).reshape(7,1)

        # Calculate wrist quat
        r = R.from_dcm(T_shoulder_gripper_center[0:3, 0:3])
        wrist_quat = r.as_quat()
        #print(wrist_quat)
        # BE CAREFUL!! Need to convert from xyzw to wxyz

        fingertip1[3:] = np.array([wrist_quat[3], wrist_quat[0], wrist_quat[1], wrist_quat[2]]).reshape(4, 1)
        fingertip2[3:] = np.array([wrist_quat[3], wrist_quat[0], wrist_quat[1], wrist_quat[2]]).reshape(4, 1)

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

        # th0 = np.array([np.pi / 4, np.pi / 4, 3 * np.pi / 4, 3 * np.pi / 4])
        #    th = kinematicConstraint(th0,P_C_Body, Body_R_C, P_E_Body, Body_R_E, P_H_Body, Body_R_H, P_K_Body, Body_R_K )

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


            #theta = fsolve(kinematicConstraint, th0, args=args, xtol=0.004)

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
    def gripper_2d_two_finger_ik_noOffset(T_Toe1, T_Toe2):
        robot_consts = consts['SCALER_climbing_6DoF'] # TODO: Should we use SCALER_climbing_6DoF_gripper here??
        IJ_X = robot_consts.L_GRIPPER_2D_L_IJ_X
        IJ_Y = robot_consts.L_GRIPPER_2D_L_IJ_Y

        L0 = robot_consts.L_GRIPPER_2D_L0
        L1_origin = robot_consts.L_GRIPPER_2D_L1
        L1 = 0.0
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

        width = np.linalg.norm(P_M_Body-P_N_Body)
        L_act = 0

        def kinematicConstraint(x_D, *args):
            y_D, width = args
            x_D = x_D[0]

            r_BD = np.sqrt((x_D - L1) ** 2 + (y_D - L2) ** 2)

            theta3 = np.arccos((r_BD ** 2 + L3 ** 2 - L4 ** 2) / (2 * r_BD * L3)) + np.arctan2(
                y_D - L2, x_D - L1)

            theta4 = theta3 + np.arccos((L3 ** 2 + L4 ** 2 - r_BD ** 2) / (2 * L3 * L4)) - np.pi

            r_FD = np.sqrt((x_D - L1) ** 2 + (y_D + L7) ** 2)
            theta6 = np.arctan2(y_D + L7, x_D - L1) - np.arccos(
                (r_FD ** 2 + L6 ** 2 - L5 ** 2) / (2 * r_FD * L6))
            theta5 = np.pi - np.arccos((L6 ** 2 + L5 ** 2 - r_FD ** 2) / (2 * L6 * L5)) + theta6

            x_G = L1 + L3 * np.cos(theta3) + L13 * np.cos(theta4 - np.pi)
            y_G = L2 + L3 * np.sin(theta3) + L13 * np.sin(theta4 - np.pi)

            r_IG = np.sqrt((x_G - L_act) ** 2 + (y_G - L0 / 2) ** 2)
            theta11 = np.arccos((r_IG ** 2 + L11 ** 2 - L12 ** 2) / (2 * r_IG * L11)) + np.arctan2(
                y_G - L0 / 2, x_G - L_act)
            theta12 = theta11 + np.arccos(
                (L11 ** 2 + L12 ** 2 - r_IG ** 2) / (2 * L11 * L12)) - np.pi

            x_L = L1 + L6 * np.cos(theta6) + L10 * np.cos(theta5 - np.pi)
            y_L = -L7 + L6 * np.sin(theta6) + L10 * np.sin(theta5 - np.pi)

            r_JK = np.sqrt((x_L - L_act) ** 2 + (y_L + L0 / 2) ** 2)
            theta8 = np.arctan2(y_L + L0 / 2, x_L - L_act) - np.arccos(
                (r_JK ** 2 + L8 ** 2 - L9 ** 2) / (2 * r_JK * L8))
            theta9 = np.pi - np.arccos((L8 ** 2 + L9 ** 2 - r_JK ** 2) / (2 * L8 * L9)) + theta8

            theta10 = theta5
            theta13 = theta4
            theta14 = theta12
            theta15 = theta9

            # Point M
            P_M = np.array([[(L_act + L11*np.cos(theta11) + (L12+ L14)*np.cos(theta12))],
                            [(L0/2 + L11*np.sin(theta11) + (L12+ L14)*np.sin(theta12))]])

            # Point N
            P_N = np.array([[L_act + L8 * np.cos(theta8) + (L9 + L15) * np.cos(theta9)],
                            [-L0 / 2 + L8 * np.sin(theta8) + (L9 + L15) * np.sin(theta9)]])

            F = P_M[1]- P_N[1] - width

            return F

        x_init = np.array([0.0])
        #width = 110
        args = (0, width)
        #x_D = fsolve(kinematicConstraint, x_init, args=args, xtol=0.004)
        x_D = root(kinematicConstraint,x_init,args=args,method='lm')
        x_D = x_D['x'][0]  # TODO: DOUBLE CHECK
        y_D = 0

        r_BD = np.sqrt((x_D - L1) ** 2 + (y_D - L2) ** 2)
        theta3 = np.arccos((r_BD ** 2 + L3 ** 2 - L4 ** 2) / (2 * r_BD * L3)) + np.arctan2(y_D - L2,
                                                                                                          x_D - L1)
        theta4 = theta3 + np.arccos((L3 ** 2 + L4 ** 2 - r_BD ** 2) / (2 * L3 * L4)) - np.pi

        x_G = L1 + L3 * np.cos(theta3) + L13 * np.cos(theta4 - np.pi)
        y_G = L2 + L3 * np.sin(theta3) + L13 * np.sin(theta4 - np.pi)

        r_IG = np.sqrt((x_G - L_act) ** 2 + (y_G - L0 / 2) ** 2)
        theta11 = np.arccos((r_IG ** 2 + L11 ** 2 - L12 ** 2) / (2 * r_IG * L11)) + np.arctan2(
            y_G - L0 / 2, x_G - L_act)
        theta12 = theta11 + np.arccos((L11 ** 2 + L12 ** 2 - r_IG ** 2) / (2 * L11 * L12)) - np.pi

        r_FD = np.sqrt((x_D - L1) ** 2 + (y_D + L7) ** 2)
        theta6 = np.arctan2(y_D + L7, x_D - L1) - np.arccos(
            (r_FD ** 2 + L6 ** 2 - L5 ** 2) / (2 * r_FD * L6))
        theta5 = np.pi - np.arccos((L6 ** 2 + L5 ** 2 - r_FD ** 2) / (2 * L6 * L5)) + theta6

        x_L = L1 + L6 * np.cos(theta6) + L10 * np.cos(theta5 - np.pi)
        y_L = -L7 + L6 * np.sin(theta6) + L10 * np.sin(theta5 - np.pi)

        r_JK = np.sqrt((x_L - L_act) ** 2 + (y_L + L0 / 2) ** 2)
        theta8 = np.arctan2(y_L + L0 / 2, x_L - L_act) - np.arccos(
            (r_JK ** 2 + L8 ** 2 - L9 ** 2) / (2 * r_JK * L8))
        theta9 = np.pi - np.arccos((L8 ** 2 + L9 ** 2 - r_JK ** 2) / (2 * L8 * L9)) + theta8

        # Point M
        P_M = np.array([[L_act + L11 * np.cos(theta11) + (L12 + L14) * np.cos(theta12)],
                        [L0 / 2 + L11 * np.sin(theta11) + (L12 + L14) * np.sin(theta12)]])

        # Point N
        P_N = np.array([[L_act + L8 * np.cos(theta8) + (L9 + L15) * np.cos(theta9)],
                        [-L0 / 2 + L8 * np.sin(theta8) + (L9 + L15) * np.sin(theta9)]])


        center_gripper = np.array([-L1_origin, 0])
        Linear_actuator = x_D - center_gripper[0]
        # print('gripper IK test')
        # pdb.set_trace()
        offset_theta = 0.0

        T_toe1_center_local = np.array([[1, 0, 0, P_M[0][0]], [0, 1, 0, P_M[1][0]], [0, 0, 1, 0.0], [0, 0, 0, 1]])
        T_toe2_center_local = np.array([[1, 0, 0, P_N[0][0]], [0, 1, 0, P_N[1][0]], [0, 0, 1, 0.0], [0, 0, 0, 1]])

        # rotation from finger to center along finger y axis by + 90 deg
        th0 = -90 / 180 * np.pi
        th1 = 90 / 180 * np.pi
        rot_y0 = np.array(([np.cos(th0), 0, np.sin(th0)], [0, 1, 0], [-np.sin(th0), 0, np.cos(th0)]))
        rot_y1 = np.array(([np.cos(th1), 0, np.sin(th1)], [0, 1, 0], [-np.sin(th1), 0, np.cos(th1)]))
        rot_center_center = np.array(([0, 0, 1], [0, 1, 0], [-1, 0, 0]))

        # check0 = util.rotation_2_euler(rot_center_center)
        T_rot_center_center = np.array([[rot_center_center[0, 0], rot_center_center[0, 1], rot_center_center[0, 2], 0],
                                        [rot_center_center[1, 0], rot_center_center[1, 1], rot_center_center[1, 2], 0],
                                        [rot_center_center[2, 0], rot_center_center[2, 1], rot_center_center[2, 2], 0],
                                        [0, 0, 0, 1]])

        T0 = np.array([[rot_y0[0, 0], rot_y0[0, 1], rot_y0[0, 2], 0],
                        [rot_y0[1, 0], rot_y0[1, 1], rot_y0[1, 2], 0],
                        [rot_y0[2, 0], rot_y0[2, 1], rot_y0[2, 2], 0],
                        [0, 0, 0, 1]])

        T1 = np.array([[rot_y1[0, 0], rot_y1[0, 1], rot_y1[0, 2], 0],
                        [rot_y1[1, 0], rot_y1[1, 1], rot_y1[1, 2], 0],
                        [rot_y1[2, 0], rot_y1[2, 1], rot_y1[2, 2], 0],
                        [0, 0, 0, 1]])

        # TODO: GET FINGERTIP POSITION FROM HERE (relative to center)
        # finger 1
        T_toe1_center = np.matmul(T_rot_center_center, T_toe1_center_local)

        # finger 2
        T_toe2_center = np.matmul(T_rot_center_center, T_toe2_center_local)


        T_toe_body = np.array([[Body_R_ToeM[0, 0], Body_R_ToeM[0, 1], Body_R_ToeM[0, 2], P_M_Body[0][0]],
                               [Body_R_ToeM[1, 0], Body_R_ToeM[1, 1], Body_R_ToeM[1, 2], P_M_Body[1][0]],
                               [Body_R_ToeM[2, 0], Body_R_ToeM[2, 1], Body_R_ToeM[2, 2], P_M_Body[2][0]],
                               [0, 0, 0, 1]])

        T_center_body_new = np.dot(np.dot(np.dot(T_toe_body , T0),  np.linalg.inv((T_toe1_center_local))) , T1)


        return Linear_actuator, offset_theta, T_center_body_new, T_toe1_center, T_toe2_center


class SCALERv1UtilMethods:
    """
    """


class SCALERv2UtilMethods:
    """ SCALERv2UtilMethods Class
    This Class contains Utility Methods for SCALER v2. These methods should be standardized methods that are applicable
    to various configurations of SCALER v2 (i.e., walking configuration [3DoF and 4DoF], climbing configuration).
    """

    @staticmethod
    def leg_v2_ik_direct(px_wrist_leg, py_wrist_leg, l_leg_link_1, l_leg_link_2, l_leg_link_a23_wrist, l_blsp,
                         leg_gamma_off_angle, leg_theta_1_off_angle):
        """ Leg v2 Inverse Kinematic Direct
        This method calculates the direct inverse kinematics of the parallel leg mechanism of SCALER v2. This is a
        standardized component that solves the joint angles of the parallel leg given the wrist location w.r.t the leg
        reference frame.
        Args:
            px_wrist_leg: X component of the wrist location w.r.t the leg reference frame. [units: mm]
            py_wrist_leg: Y component of the wrist location w.r.t the leg reference frame. [units: mm]
            l_leg_link_1: Length of the Leg Link 1 (Links from the Top/Bottom Leg Servos to the Elbow Joints).
            [units: mm]
            l_leg_link_2: Length of the Leg Link 2 (Links from the Elbow Joints to the Hinge Joint). [units: mm]
            l_leg_link_a23_wrist: Length of the Leg Link from the Hinge Joint to the Wrist Joint. [units: mm]
            l_blsp: Length Between the Top and Bottom Leg Servos [units: mm]
            leg_gamma_off_angle:
            leg_theta_1_off_angle:
        Returns:
            q11             : Angle of the Top Leg Servo Joint [units: radians]
            q12             : Angle of the Top Leg Elbow Joint [units: radians]
            q13             : Angle to satisfy five-bar closed chain (q13 = q21 + q22 - q11 - q12) [units: radians]
            q21             : Angle of the Bottom Leg Servo Joint [units: radians]
            q22             : Angle of the Bottom Leg Elbow Joint [units: radians]
        """
        # Constant Coefficients (Bottom Leg Links)
        a2 = -2 * l_leg_link_1 * (px_wrist_leg - (l_blsp / 2))
        b2 = -2 * l_leg_link_1 * py_wrist_leg
        c2 = (px_wrist_leg - (l_blsp / 2)) ** 2 + py_wrist_leg ** 2 + l_leg_link_1 ** 2 - l_leg_link_2 ** 2 - \
            l_leg_link_a23_wrist ** 2 - 2 * l_leg_link_2 * l_leg_link_a23_wrist * np.cos(leg_gamma_off_angle)

        q21 = 2 * np.arctan2((-b2 + np.sqrt(b2 ** 2 - c2 ** 2 + a2 ** 2)),
                             (c2 - a2))

        q22 = np.arctan2(py_wrist_leg - l_leg_link_1 * np.sin(q21),
                         px_wrist_leg - (l_blsp / 2) - l_leg_link_1 * np.cos(q21)) - q21 - leg_theta_1_off_angle

        phi = q21 + q22

        x_a23_wrist = l_leg_link_a23_wrist * np.cos(phi + leg_gamma_off_angle)
        y_a23_wrist = l_leg_link_a23_wrist * np.sin(phi + leg_gamma_off_angle)

        a1 = -2 * l_leg_link_1 * (px_wrist_leg - x_a23_wrist - (-l_blsp / 2))
        b1 = 2 * l_leg_link_1 * (y_a23_wrist - py_wrist_leg)
        c1 = l_leg_link_a23_wrist ** 2 + (px_wrist_leg - (-l_blsp / 2)) ** 2 + 2 * x_a23_wrist * (-l_blsp / 2) - \
            2 * x_a23_wrist * px_wrist_leg - 2 * y_a23_wrist * py_wrist_leg + py_wrist_leg ** 2 + \
            l_leg_link_1 ** 2 - l_leg_link_2 ** 2

        q11 = 2 * np.arctan2((-b1 - np.sqrt(b1 ** 2 - c1 ** 2 + a1 ** 2)),
                             (c1 - a1))

        q12 = np.arctan2(py_wrist_leg - l_leg_link_1 * np.sin(q11) - y_a23_wrist,
                         px_wrist_leg - (-l_blsp / 2) - l_leg_link_1 * np.cos(q11) - x_a23_wrist) - q11

        q13 = q21 + q22 - q11 - q12

        return q11, q12, q13, q21, q22, phi

