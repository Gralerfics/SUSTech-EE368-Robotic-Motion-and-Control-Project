#ifndef ARUCO_H
#define ARUCO_H

#include <memory>

#include "compute.h"
#include "camera.h"

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>

class ArucoDetector {
private:
    float marker_size;
    cv::Ptr<cv::aruco::Dictionary> dictionary = nullptr;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = nullptr;
    std::shared_ptr<RealSense> camera = nullptr;

public:
    ArucoDetector(std::shared_ptr<RealSense> camera, cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary, float marker_size);
    ~ArucoDetector() {}

    void Detect(cv::Mat &image, std::vector<int> &ids, std::vector<Vector3f> &rvecs, std::vector<Vector3f> &tvecs, bool draw = false);
};

class ArucoMarker {
private:
    int id;
    int num;
    std::vector<Vector6f> rt_vecs;
    std::vector<float> weights;
    Matrix4f T_M_P;

public:
    ArucoMarker(int id, int num = 50, Matrix4f T_M_P = Matrix4f::Identity());
    ~ArucoMarker() {}

    int GetId() { return id; }
    Matrix4f GetT_M_P() { return T_M_P; }
    void SetT_M_P(Vector3f rvec, Vector3f tvec);

    void Update(Vector3f rvec, Vector3f tvec);
    Vector6f GetRtVec();
};

#endif