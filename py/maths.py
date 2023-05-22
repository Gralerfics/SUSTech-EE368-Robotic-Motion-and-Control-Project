import numpy as np
from scipy.spatial.transform import Rotation


def quat2dcm(quat):
    return Rotation.from_quat(quat).as_dcm()


def rotvec2dcm(rotvec):
    return Rotation.from_rotvec(rotvec).as_dcm()


# def dcm2euler(R, degrees = True):
#     return Rotation.from_dcm(R).as_euler('xyz', degrees = degrees)


def euler2dcm(euler, degrees = True):
    return Rotation.from_euler('xyz', euler, degrees = degrees).as_dcm()


def RP2T(R, P):
    return np.vstack((np.hstack((R, np.array(P).reshape(3, 1))), [0, 0, 0, 1]))


def rtvecs2T(rtvecs):
    R, P = rotvec2dcm(rtvecs[0:3]), np.array(rtvecs[3:6])
    return RP2T(R, P)


def pose2T(pose):
    R, P = euler2dcm(pose[3:6]), np.array(pose[0:3])
    return RP2T(R, P)

