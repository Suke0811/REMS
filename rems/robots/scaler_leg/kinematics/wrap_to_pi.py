import numpy as np

def wrap_to_pi(rad):
    """
    Wrap the angle (in radiants) to within [-pi, pi)
    :param rad:
    :return:
    """
    ret_rad = rad[:]

    for iter in range(len(ret_rad)):

        while ret_rad[iter] >= np.pi:
            ret_rad[iter] -= 2.0*np.pi

        while ret_rad[iter] < -np.pi:
            ret_rad[iter] += 2.0*np.pi

    return ret_rad


def wrap_to_2pi(rad):
    """
    Wrap the angle (in radiants) to within [0, 2*pi)
    :param rad:
    :return:
    """
    ret_rad = rad[:]

    for iter in range(len(ret_rad)):

        while ret_rad[iter] >= 2*np.pi:
            ret_rad[iter] -= 2.0*np.pi

        while ret_rad[iter] < 0:
            ret_rad[iter] += 2.0*np.pi

    return ret_rad
