import numpy as np
from scipy.spatial.transform import Rotation as R

def euler2d_2_quat(theta, degrees=False):
    rot = R.from_euler('xyz',[0,0,theta],degrees=degrees)
    return rot.as_quat()

