#ifndef CAMERA_H
#define CAMERA_H

#include <memory>

#include "compute.h"

#include <librealsense2/rs.hpp>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

class RealSense {
private:
    int frame_width = 640;
    int frame_height = 480;
    std::shared_ptr<rs2::pipeline> pipeline = nullptr;
    std::shared_ptr<rs2::config> config = nullptr;

public:
    RealSense();
    ~RealSense();

    cv::Mat GetColorFrame();
    cv::Mat GetKForOpenCV();
    cv::Mat GetDForOpenCV();

    Matrix3f K;
    Vector5f D;
};

#endif