#include "compute.h"

Matrix3f quat2dcm(Quaternionf quat) {
    return quat.toRotationMatrix();
}

Matrix3f rotvec2dcm(Vector3f rotvec) {
    float norm = rotvec.norm();
    AngleAxisf ret(norm, rotvec / norm);
    return ret.toRotationMatrix();
}

Matrix3f euler2dcm(Vector3f euler, bool degree) {
    Matrix3f ret;
    if (degree) {
        ret = AngleAxisf(euler(2) / 180.0 * M_PI, Vector3f::UnitZ())
            * AngleAxisf(euler(1) / 180.0 * M_PI, Vector3f::UnitY())
            * AngleAxisf(euler(0) / 180.0 * M_PI, Vector3f::UnitX());
    } else {
        ret = AngleAxisf(euler(2), Vector3f::UnitZ())
            * AngleAxisf(euler(1), Vector3f::UnitY())
            * AngleAxisf(euler(0), Vector3f::UnitX());
    }
    return ret;
}

Matrix4f Rt2T(Matrix3f R, Vector3f t) {
    Eigen::Matrix4f ret = Eigen::Matrix4f::Identity();
    ret.topLeftCorner<3, 3>() = R;
    ret.topRightCorner<3, 1>() = t;
    return ret;
}

Matrix4f RtVec2T(Vector6f rt_vec) {
    Matrix3f R = rotvec2dcm(rt_vec.head<3>());
    Vector3f t = rt_vec.tail<3>();
    return Rt2T(R, t);
}

Matrix4f pose2T(Vector6f pose, bool degree) {
    Matrix3f R = euler2dcm(pose.tail<3>(), degree);
    Vector3f t = pose.head<3>();
    return Rt2T(R, t);
}
