import numpy as np
from scipy.spatial.transform import Rotation


def quat2dcm(quat):
    return Rotation.from_quat(quat).as_dcm()


def rotvec2dcm(rotvec):
    return Rotation.from_rotvec(rotvec).as_dcm()


def dcm2euler(R, degrees = True):
    return Rotation.from_dcm(R).as_euler('xyz', degrees = degrees)


def RP2T(R, P):
    return np.vstack((np.hstack((R, P.reshape(3, 1))), [0, 0, 0, 1]))

