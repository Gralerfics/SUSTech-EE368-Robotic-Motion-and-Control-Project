#include "aruco.h"


ArucoDetector::ArucoDetector(std::shared_ptr<RealSense> camera, cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name){
    this->camera = camera;
    dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);
    parameters = cv::aruco::DetectorParameters::create();
}

ArucoDetector::~ArucoDetector() {
}

void ArucoDetector::Detect(cv::Mat &image, float marker_length, std::vector<int> &ids, std::vector<Vector3f> &rvecs, std::vector<Vector3f> &tvecs, bool draw) {
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
    cv::aruco::detectMarkers(image, dictionary, marker_corners, ids, parameters, rejected_candidates, camera->GetKForOpenCV(), camera->GetDForOpenCV());
    
    if (ids.size() > 0) {
        std::vector<cv::Vec3d> rvecs_cv, tvecs_cv;
        cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length, camera->GetKForOpenCV(), camera->GetDForOpenCV(), rvecs_cv, tvecs_cv);

        rvecs.clear();
        tvecs.clear();
        for (auto i = 0; i < ids.size(); i ++) {
            Vector3f rvec_eigen, tvec_eigen;
            rvec_eigen(0) = rvecs_cv[i][0];
            rvec_eigen(1) = rvecs_cv[i][1];
            rvec_eigen(2) = rvecs_cv[i][2];
            tvec_eigen(0) = tvecs_cv[i][0];
            tvec_eigen(1) = tvecs_cv[i][1];
            tvec_eigen(2) = tvecs_cv[i][2];
            rvecs.push_back(rvec_eigen);
            tvecs.push_back(tvec_eigen);
            if (draw) cv::aruco::drawAxis(image, camera->GetKForOpenCV(), camera->GetDForOpenCV(), rvecs_cv[i], tvecs_cv[i], 0.1);
        }
        if (draw) cv::aruco::drawDetectedMarkers(image, marker_corners, ids);
    }
}

ArucoMarker::ArucoMarker(int id, int num = 100) {
    this->id = id;
    this->num = num;
    Vector6f zero_vec;
    zero_vec << 0, 0, 0, 0, 0, 0;
    for (auto i = 0; i < num; i ++) {
        rt_vecs.push_back(zero_vec);
        weights.push_back(1 / num);
    }
}

ArucoMarker::~ArucoMarker() {
}

// void ArucoMarker::Update(Vector3f rvec, Vector3f tvec) {
//     Vector6f rt_vec;
//     rt_vec << rvec(0), rvec(1), rvec(2), tvec(0), tvec(1), tvec(2);
//     // Predict
//     Vector6f current = GetRTVecs();
//     Vector6f increasement = rt_vec - current;
//     for (auto i = 0; i < num; i ++) {
//         rt_vecs[i] += increasement;
//     }
//     // Update
//     float sum = 0.0;
//     for (auto i = 0; i < num; i ++) {
//         float norm = (current - rt_vecs[i]).norm();
//         if norm < 1e-6 {
//             weights[i] = 1e6;
//         } else {
//             weights[i] = 1.0 / norm;
//         }
//         sum += weights[i];
//     }
//     // Resample

// }

// Vector6f ArucoMarker::GetRTVecs() {
//     Vector6f average;
//     for (auto i = 0; i < num; i ++) {
//         average += rt_vecs[i] * weights[i];
//     }
//     return average;
// }
