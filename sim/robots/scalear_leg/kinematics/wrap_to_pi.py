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
