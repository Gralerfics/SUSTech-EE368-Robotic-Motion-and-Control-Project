import numpy as np
from scipy.spatial.transform import Rotation


def quat2dcm(quat):
    return Rotation.from_quat(quat).as_dcm()


def rotvec2dcm(rotvec):
    return Rotation.from_rotvec(rotvec).as_dcm()


def dcm2euler(R, degrees = True):
    return Rotation.from_dcm(R).as_euler('xyz', degrees = degrees)


def col(vec):
    return np.array(vec).reshape(len(vec), 1)


def col2row(vec):
    return np.array(vec).T[0]


def col2homo(vec):
    return np.vstack((vec, [1]))


def RP2T(R, P):
    return np.vstack((np.hstack((R, P)), [0, 0, 0, 1]))

