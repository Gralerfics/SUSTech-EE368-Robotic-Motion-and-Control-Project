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
    cv::Ptr<cv::aruco::Dictionary> dictionary = nullptr;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = nullptr;
    std::shared_ptr<RealSense> camera = nullptr;

public:
    ArucoDetector(std::shared_ptr<RealSense> camera, cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary);
    ~ArucoDetector() {}

    void Detect(cv::Mat &image, float marker_length, std::vector<int> &ids, std::vector<Vector3f> &rvecs, std::vector<Vector3f> &tvecs, bool draw = false);
};

class ArucoMarker {
private:
    int id;
    int num;
    std::vector<Vector6f> rt_vecs;
    std::vector<float> weights;

public:
    ArucoMarker(int id, int num = 50);
    ~ArucoMarker() {}

    int GetId() { return id; }
    
    void Update(Vector3f rvec, Vector3f tvec);
    Vector6f GetRtVec();
};

#endif