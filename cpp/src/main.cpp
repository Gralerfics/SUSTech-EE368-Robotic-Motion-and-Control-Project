#include "arm.h"
#include "camera.h"
#include "aruco.h"

void delay(double s) {
    std::this_thread::sleep_for(std::chrono::microseconds((int) (s * 1000000)));
}

int main(int argc, char **argv) {
    std::shared_ptr<Gen3Lite> arm(new Gen3Lite());
    std::shared_ptr<RealSense> cam(new RealSense());
    ArucoDetector detector(cam, cv::aruco::DICT_ARUCO_ORIGINAL);
    ArucoMarker marker(1);

    arm->MoveArmToHome();
    arm->SetGripperPosition(1.0);

    while (true) {
        cv::Mat frame = cam->GetColorFrame();

        std::vector<int> ids;
        std::vector<Vector3f> rvecs, tvecs;
        detector.Detect(frame, 0.05, ids, rvecs, tvecs, true);

        printf("Detected %d markers\n", ids.size());
        for (auto i = 0; i < ids.size(); i++) {
            printf("Marker %d: (%f, %f, %f)\n", ids[i], tvecs[i](0), tvecs[i](1), tvecs[i](2));
        }

        cv::imshow("Image", frame);
        int key = cv::waitKey(1);
        if (key == 'q') break;
    }
}
