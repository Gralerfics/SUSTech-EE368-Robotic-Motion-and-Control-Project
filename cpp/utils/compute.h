#ifndef COMPUTE_H
#define COMPUTE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

typedef Matrix<float, 5, 1> Vector5f;
typedef Matrix<float, 6, 1> Vector6f;

Matrix3f quat2dcm(Quaternionf quat);

Matrix3f rotvec2dcm(Vector3f rotvec);

Vector3f dcm2rotvec(Matrix3f dcm);

Matrix3f euler2dcm(Vector3f euler, bool degree = true);

Matrix4f Rt2T(Matrix3f R, Vector3f t);

Matrix4f RtVec2T(Vector6f rt_vec);

Matrix4f pose2T(Vector6f pose, bool degree = true);

#endif