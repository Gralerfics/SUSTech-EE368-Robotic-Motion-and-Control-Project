#include "camera.h"

RealSense::RealSense() {
    pipeline = std::make_shared<rs2::pipeline>();
    config = std::make_shared<rs2::config>();
    config->enable_stream(RS2_STREAM_COLOR, frame_width, frame_height, RS2_FORMAT_BGR8, 30);
    pipeline->start();

    rs2::frameset frames = pipeline->wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();
    rs2_intrinsics intrinsics = color_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    K <<
        intrinsics.fx, 0, intrinsics.ppx,
        0, intrinsics.fy, intrinsics.ppy,
        0, 0, 1;
    D << intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4];
}

RealSense::~RealSense() {
    pipeline->stop();
}

cv::Mat RealSense::GetColorFrame() {
    rs2::frameset frames = pipeline->wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();
    cv::Mat color_image(cv::Size(frame_width, frame_height), CV_8UC3, (void *) color_frame.get_data(), cv::Mat::AUTO_STEP);
    return color_image;
}

cv::Mat RealSense::GetKForOpenCV() {
    cv::Mat ret;
    cv::eigen2cv(K, ret);
    return ret;
}

cv::Mat RealSense::GetDForOpenCV() {
    cv::Mat ret;
    cv::eigen2cv(D, ret);
    return ret;
}
