import os
import sys


from .hardware_constants import consts, list_name_robots, division_factor
import numpy as np
from scipy.spatial.transform import Rotation as R
#data_name = glob.read_one_data(glob.mem_settings, 'robot_name')
#robot_name = list_name_robots[int(data_name)]
robot_name = 'SCALER_climbing_6DoF'
robot_consts = consts[robot_name]


def getCtrlPointTwoWall(whichLeg, Stroke, liftHeight):
    """ Generate control points for interpolation """
    halfZstroke = Stroke[2]/2.0
    X_stroke = Stroke[0]
    Y_stroke = Stroke[1]

    x_controlPoint_offset = 40.0/division_factor  # Top control point height above liftheight

    # Pick up leg and make a full curve down to a upper position of wall
    #        3. (liftheight, 3*half_Zstroke)
    #       /                 ^
    #      /                  |
    # goal        1. (liftHeight+x_controlPoint_offset, 2*halfZstroke)
    #                         ^
    #                         |
    #        0. (liftheight, half_Zstroke)
    #       /
    #      /
    # curr

    if (whichLeg == 0 or whichLeg == 1):  # Right side legs
        # not sure if the following codes are correct. Could Xuan try to find the time to investigate this?
        if whichLeg == 0: #RF leg
            ctrlPoint = [[X_stroke-liftHeight,                          0.0,         0*halfZstroke],  #TODO: was halfZstroke for trajectories that are not Footsteps_StepTransition2
                         [X_stroke-(liftHeight+x_controlPoint_offset),  Y_stroke/2,  1*halfZstroke],  #TODO: was 2*halfZstroke for trajectories that are not Footsteps_StepTransition2
                         [X_stroke-(liftHeight+x_controlPoint_offset),  Y_stroke,    2*halfZstroke],
                         [X_stroke-liftHeight,                          Y_stroke,    3*halfZstroke]]

        else: #RB leg
            ctrlPoint = [[X_stroke-liftHeight,                          0.0,         0*halfZstroke],
                         [X_stroke-(liftHeight+x_controlPoint_offset),  Y_stroke/2,  1*halfZstroke],
                         [X_stroke-(liftHeight+x_controlPoint_offset),  Y_stroke,    2*halfZstroke],
                         [X_stroke-liftHeight,                          Y_stroke,    3*halfZstroke]]


    else:  # Left side legs
        if whichLeg == 3: #LF leg
            ctrlPoint = [[X_stroke+liftHeight,                        0.0,         0*halfZstroke],
                         [X_stroke+liftHeight+x_controlPoint_offset,  Y_stroke/2,  1*halfZstroke],
                         [X_stroke+liftHeight+x_controlPoint_offset,  Y_stroke,    2*halfZstroke],
                         [X_stroke+liftHeight,                        Y_stroke,    3*halfZstroke]]
        else: #LB leg
            ctrlPoint = [[X_stroke+liftHeight,                         0.0,          0*halfZstroke],
                         [X_stroke+liftHeight+x_controlPoint_offset,   Y_stroke/2,   1*halfZstroke],
                         [X_stroke+liftHeight+x_controlPoint_offset,   Y_stroke,     2*halfZstroke],
                         [X_stroke+liftHeight,                         Y_stroke,     3*halfZstroke]]

    return ctrlPoint


def getCtrlPointOneWall(whichLeg, Stroke, liftHeight):
    """
    This function assumes B-spline interpolation
    :param whichLeg:
    :param Stroke:
    :param liftHeight:
    :return:
    """

    halfXstroke = Stroke[0]/2.0
    halfYstroke = Stroke[1]/2.0
    X_stroke = Stroke[0]
    Y_stroke = Stroke[1]

    z_controlPoint_offset = 40.0/division_factor  # Top control point height above liftheight

    # Pick up leg and make a full curve down to a upper position of wall
    #        3. (liftheight, 3*half_Zstroke)
    #       /                 ^
    #      /                  |
    # goal        1. (liftHeight+x_controlPoint_offset, 2*halfZstroke)
    #                         ^
    #                         |
    #        0. (liftheight, half_Zstroke)
    #       /
    #      /
    # curr

    # not sure if the following codes are correct. Could Xuan try to find the time to investigate this?
    ctrlPoint = [[0*halfXstroke, 0*halfYstroke, liftHeight],
                 [1*halfXstroke, 1*halfYstroke, liftHeight+z_controlPoint_offset],
                 [2*halfXstroke, 2*halfYstroke, liftHeight+z_controlPoint_offset],
                 [3*halfXstroke, 3*halfYstroke, liftHeight]]

    return ctrlPoint

def getCtrlPointWalk(whichLeg, Stroke, liftHeight):
    """
    This function assumes linear triangular interpolation
    :param whichLeg:
    :param Stroke:
    :param liftHeight:
    :return:
    """

    halfXstroke = Stroke[0]/2.0
    halfYstroke = Stroke[1]/2.0

    # not sure if the following codes are correct. Could Xuan try to find the time to investigate this?
    ctrlPoint = [halfXstroke, halfYstroke, liftHeight]

    return ctrlPoint

def tf_matrix_from_dh_parameters(alpha, theta, a, d):
    """ Calculated the Transformation Matrix from the provided Modified Denavit-Hartenberg Parameters

    Args:
        alpha: The Link Twist angle [units: radians]

        theta: The Link Angle [units: radians]

        a: The Link Length [units: (i.e., mm)]

        d: The Link Offset [units: (i.e., mm)]

    Returns:
        tf_matrix: This is the transformation matrix

            tf_matrix = [[           cos(theta)               -sin(theta)                 0                     a ],
                         [ sin(theta)cos(alpha)      cos(theta)cos(alpha)       -sin(alpha)         -sin(alpha)*d ],
                         [ sin(theta)sin(alpha)      cos(theta)sin(alpha)        cos(alpha)          cos(alpha)*d ],
                         [                    0                         0                 0                     1 ]]

    """
    tf_matrix = np.array([[np.cos(theta), -np.sin(theta), 0, a],
                          [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha),
                           -np.sin(alpha), -np.sin(alpha) * d],
                          [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha),
                           np.cos(alpha), np.cos(alpha) * d],
                          [0, 0, 0, 1]])

    return tf_matrix


def quaternion_2_rotation_matrix(quaternion):
    """ Quaternion 2 Rotation Matrix

    This method converts a quaternion to a spatial rotation matrix

    Args:
        quaternion: Orientation represented as a quaternion vector of form q = [w, x, y, z], with w as the scalar
                    number. [dim: 1 x 4]

    Returns:
        rotation_matrix: Orientation represented as an orthonormal rotation matrix. [dim: 3 x 3]

    """
    w = quaternion[0]
    x = quaternion[1]
    y = quaternion[2]
    z = quaternion[3]

    # Normalize if the quaternion is not a unit quaternion.
    s = np.linalg.norm(quaternion) ** -2

    rotation_matrix = np.array([[1 - 2 * s * (y ** 2 + z ** 2), 2 * s * (x * y - z * w), 2 * s * (x * z + y * w)],
                                [2 * s * (x * y + z * w), 1 - 2 * s * (x ** 2 + z ** 2), 2 * s * (y * z - x * w)],
                                [2 * s * (x * z - y * w), 2 * s * (y * z + x * w), 1 - 2 * s * (x ** 2 + y ** 2)]])

    return rotation_matrix


def unpack_rotation_matrix(rotation_matrix):
    """ Unpack Rotation Matrix

    Args:
        rotation_matrix: Orientation represented as an orthonormal rotation matrix. [dim: 3 x 3]

    Returns:
        r_11, r_12, r_13,
        r_21, r_22, r_23,
        r_31, r_32, r_33: The elements of the Rotation Matrix

    """
    r_11 = rotation_matrix[0][0]
    r_12 = rotation_matrix[0][1]
    r_13 = rotation_matrix[0][2]

    r_21 = rotation_matrix[1][0]
    r_22 = rotation_matrix[1][1]
    r_23 = rotation_matrix[1][2]

    r_31 = rotation_matrix[2][0]
    r_32 = rotation_matrix[2][1]
    r_33 = rotation_matrix[2][2]

    return r_11, r_12, r_13, r_21, r_22, r_23, r_31, r_32, r_33


def rotation_matrix_2_quaternion(rotation_matrix):
    """ Rotation Matrix 2 Quaternion

    This method converts spatial rotation matrices to quaternions

    Args:
        rotation_matrix: Orientation represented as an orthonormal rotation matrix. [dim: 3 x 3]

    Returns:
        quaternion: Orientation represented as a quaternion vector of form q = [w, x, y, z], with w as the scalar
                    number. [dim: 1 x 4]

    """
    r_11, r_12, r_13, r_21, r_22, r_23, r_31, r_32, r_33 = unpack_rotation_matrix(rotation_matrix)

    # q0_magnitude = np.sqrt((1 + r_11 + r_22 + r_33) / 4)
    # q1_magnitude = np.sqrt((1 + r_11 - r_22 - r_33) / 4)
    # q2_magnitude = np.sqrt((1 - r_11 + r_22 - r_33) / 4)
    # q3_magnitude = np.sqrt((1 - r_11 - r_22 + r_33) / 4)
    #
    # print(q0_magnitude)
    # print(q1_magnitude)
    # print(q2_magnitude)
    # print(q3_magnitude)

    if r_33 < 0:
        if r_11 > r_22:
            t = 1 + r_11 - r_22 - r_33
            q_x = np.sqrt(t / 4)

            q_w = (r_32 - r_23) / (4 * q_x)
            q_y = (r_12 + r_21) / (4 * q_x)
            q_z = (r_13 + r_31) / (4 * q_x)
        else:
            t = 1 - r_11 + r_22 - r_33
            q_y = np.sqrt(t / 4)

            q_w = (r_13 - r_31) / (4 * q_y)
            q_x = (r_12 + r_21) / (4 * q_y)
            q_z = (r_23 + r_32) / (4 * q_y)
    else:
        if r_11 < -r_22:
            t = 1 - r_11 - r_22 + r_33
            q_z = np.sqrt(t / 4)

            q_w = (r_21 - r_12) / (4 * q_z)
            q_x = (r_13 + r_31) / (4 * q_z)
            q_y = (r_23 + r_32) / (4 * q_z)
        else:
            t = 1 + r_11 + r_22 + r_33
            q_w = np.sqrt(t / 4)

            q_x = (r_32 - r_23) / (4 * q_w)
            q_y = (r_13 - r_31) / (4 * q_w)
            q_z = (r_21 - r_12) / (4 * q_w)

    quaternion = np.array([q_w, q_x, q_y, q_z])
    return quaternion


def euler_2_quaternion(euler_angle):
    """ Euler angles 2 Quaternion

    This method converts euler angles (X-Y-Z "fixed" angle) to quaternions
    The Euler angle definition follows equation 2.64 in the following text book, "Introduction to Robotics Mechanics and Control Third Edition, John J. Craig"
    http://www.mech.sharif.ir/c/document_library/get_file?uuid=5a4bb247-1430-4e46-942c-d692dead831f&groupId=14040

    Args:
        euler_angle [degree as a unit]: rotation along "fixed" x, y, z [dim: 3 x 1]. This is extrinsic rotation
        So, if you want to consider rotation matrix,
        R_{from A to B} = R_{along z-axis \alpha} * R_{along y-axis \beta} * R_{along x-axis \gamma}

    Returns:
        quaternion: Orientation represented as a quaternion vector of form q = [w, x, y, z], with w as the scalar
                    number. [dim: 1 x 4]

    """
    # first rotate along fixed x axis, then rotate along fixed y axis and along fixed z axis
    r = R.from_euler('xyz', euler_angle, degrees=False)
    quaternion_xyzw = r.as_quat()
    quaternion = np.array([quaternion_xyzw[3], quaternion_xyzw[0], quaternion_xyzw[1], quaternion_xyzw[2]])
    return quaternion

def quaternion_2_euler(quaternion):
    """ quaternion 2 euler

        This method converts quaterninon to euler angles (X-Y-Z "fixed" angle)
        The Euler angle definition follows equation 2.64 in the following text book, "Introduction to Robotics Mechanics and Control Third Edition, John J. Craig"
        http://www.mech.sharif.ir/c/document_library/get_file?uuid=5a4bb247-1430-4e46-942c-d692dead831f&groupId=14040

        Args:
            quaternion: Orientation represented as a quaternion vector of form q = [w, x, y, z], with w as the scalar
                        number. [dim: 1 x 4]
        Returns:
            euler_angle [degree as a unit]: rotation along "fixed" x, y, z [dim: 3 x 1]. This is extrinsic rotation
            So, if you want to consider rotation matrix,
            R_{from A to B} = R_{along z-axis \alpha} * R_{along y-axis \beta} * R_{along x-axis \gamma}
        """

    xyzw_quat = np.array([quaternion[1],quaternion[2],quaternion[3],quaternion[0]])
    r = R.from_quat(xyzw_quat)
    euler_angle = r.as_euler('xyz')
    euler = np.array([euler_angle[0],euler_angle[1],euler_angle[2]]).reshape(3,1)

    return euler

def error_R_matrix(R_desired, R_current, zero_th=1e-5):

    assert np.shape(R_desired) == (3, 3), "Incorrect shape of rotation matrix!"
    assert np.shape(R_current) == (3, 3), "Incorrect shape of rotation matrix!"

    if np.max(abs(R_desired - R_current)) <= zero_th:
        return np.array([0.0, 0.0, 0.0]), 0.0

    else:
        Re = np.dot(R_desired.transpose(), R_current)

        theta = np.arccos(0.5*(np.trace(Re)-1))
        wx = (Re - Re.transpose())/(2*np.sin(theta))
        w = np.array([-wx[1, 2], wx[0, 2], -wx[0, 1]])

        return w, theta

def euler_2_rotation(euler_angle):
    """ Euler angles 2 rotation matrix

    This method converts euler angles (X-Y-Z "fixed" angle) to rotation matrix
    The Euler angle definition follows equation 2.64 in the following text book, "Introduction to Robotics Mechanics and Control Third Edition, John J. Craig"
    http://www.mech.sharif.ir/c/document_library/get_file?uuid=5a4bb247-1430-4e46-942c-d692dead831f&groupId=14040

    Args:
        euler_angle [degree as a unit]: rotation along "fixed" x, y, z [dim: 3 x 1]. This is extrinsic rotation
        So, if you want to consider rotation matrix,
        R_{from A to B} = R_{along z-axis \alpha} * R_{along y-axis \beta} * R_{along x-axis \gamma}

    Returns:
        rotation matrix: Orientation represented as a rotation matrix of form rotation \in R^{3 \times 3}

    """

    # first rotate along fixed x axis, then rotate along fixed y axis and along fixed z axis
    r = R.from_euler('xyz', euler_angle, degrees=False)
    rotation = r.as_matrix()

    return rotation

def rotation_2_euler(rotation):
    """ rotation matrix 2 Euler angles

    This method converts rotation matrix (X-Y-Z fixed axis) to euler angles (X-Y-Z "fixed" angle)
    The Euler angle definition follows equation 2.64 in the following text book, "Introduction to Robotics Mechanics and Control Third Edition, John J. Craig"
    http://www.mech.sharif.ir/c/document_library/get_file?uuid=5a4bb247-1430-4e46-942c-d692dead831f&groupId=14040

    Args:
        rotation matrix: Orientation represented as a rotation matrix of form rotation \in R^{3 \times 3}

    Returns:
        euler_angle [degree as a unit]: rotation along "fixed" x, y, z [dim: 3 x 1]. This is extrinsic rotation
        So, if you want to consider rotation matrix,
        R_{from A to B} = R_{along z-axis \alpha} * R_{along y-axis \beta} * R_{along x-axis \gamma}

    """

    # first rotate along fixed x axis, then rotate along fixed y axis and along fixed z axis
    #beta = np.arctan2(-rotation[2, 0], np.sqrt(rotation[0, 0]**2 + rotation[1, 0]**2))

    # TODO singularity check for beta

    #alpha = np.arctan2(rotation[1,0] / np.cos(beta), rotation[0, 0] / np.cos(beta))

    #gamma = np.arctan2(rotation[2, 1] / np.cos(beta), rotation[2, 2] / np.cos(beta))

    #euler_angle = np.array([gamma, beta, alpha])

    r = R.from_matrix(rotation)
    euler_angle = r.as_euler('xyz')

    return euler_angle

def get_another_finger(parent, dist, rot):
    parent_tmp = np.copy(parent)

    # TODO: make it more general like Yuki's ADMM code
    global_child = np.array(([0,  0,  0]))
    global_child = parent_tmp + dist * np.array(([np.sin(rot), -np.cos(rot) , 0]))

    return global_child
#
# def T_LG(pg, offset, rot):
#

