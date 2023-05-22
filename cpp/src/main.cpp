#include "arm.h"
#include "camera.h"
#include "aruco.h"
#include "control.h"
#include "compute.h"

void delay(double s) {
    std::this_thread::sleep_for(std::chrono::microseconds((int) (s * 1000000)));
}

int main(int argc, char **argv) {
    std::shared_ptr<Gen3Lite> arm(new Gen3Lite());
    std::shared_ptr<RealSense> cam(new RealSense());

    // ArucoDetector detector(cam, cv::aruco::DICT_4X4_50, 0.06);
    ArucoDetector detector(cam, cv::aruco::DICT_ARUCO_ORIGINAL, 0.05);
    // std::vector<ArucoMarker> markers(4);
    ArucoMarker marker(1);

    PIDController<Vector3f> linear_controller(1.6, 0.03, 0.4);
    PIDController<Vector2f> angular_controller(100.0, 1.0, -10.0);

    Matrix3f R_0_E = quat2dcm(Quaternionf(0.0188425, -0.0246767, 0.968246, -0.248064)); // w, x, y, z
    Vector3f t_0_E = Vector3f(0.378836, 0.380062, 0.423619);
    Matrix4f T_0_E = Rt2T(R_0_E, t_0_E);

    arm->MoveArmToHome();
    arm->SetGripperPosition(1.0);

    linear_controller.Init();
    angular_controller.Init();

    int lost_frames = 0;
    while (true) {
        cv::Mat frame = cam->GetColorFrame();

        std::vector<int> ids;
        std::vector<Vector3f> rvecs, tvecs;
        detector.Detect(frame, ids, rvecs, tvecs, true);

        if (!ids.empty()) {
            for (size_t i = 0; i < ids.size(); i ++) {
                // if (ids[i] == 1) {
                //     markers[0].Update(rvecs[i], tvecs[i]);
                // } else if ...
                if (ids[i] == marker.GetId()) {
                    marker.Update(rvecs[i], tvecs[i]);
                }
            }

            Vector6f rt_vec = marker.GetRtVec();

            Matrix4f T_E_P = RtVec2T(rt_vec);
            Matrix4f T_0_P = T_0_E * T_E_P;

            Vector6f current_pose = arm->GetToolCartesianPose();
            Vector6f current_twist = arm->GetToolTwist();
            
            Matrix4f T_0_H = pose2T(current_pose);
            Matrix4f T_H_0 = T_0_H.inverse();

            Vector4f X_P_target_homo = Vector4f(0.0, 0.0, -0.1, 1.0);
            Vector3f X_H_target = (T_H_0 * T_0_P * X_P_target_homo).block<3, 1>(0, 0);
            Vector3f V_H_target = Vector3f(0.0, 0.0, 0.0);

            Vector3f V_H_measure = T_H_0.block<3, 3>(0, 0) * current_twist.block<3, 1>(0, 0);
            Vector3f X_H_error = X_H_target;
            Vector3f V_H_error = V_H_target - V_H_measure;
            Vector3f LinearTwist = linear_controller.Step(X_H_error, V_H_error);

            Matrix3f R_H_P = T_H_0.block<3, 3>(0, 0) * T_0_P.block<3, 3>(0, 0);
            Vector3f n_H = R_H_P.block<3, 1>(0, 2);
            Vector2f n_H_error = Vector2f(n_H(2), -n_H(0));
            Vector2f AngularTwist = angular_controller.Step(n_H_error);

            Vector6f Twist;
            Twist << LinearTwist(0), LinearTwist(1), LinearTwist(2), AngularTwist(0), 0.0, AngularTwist(1);
            arm->SetTwist(Twist, k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
        } else {
            lost_frames += 1;
            if (lost_frames > 10) {
                arm->StopMotioning();
            }
        }

        cv::imshow("Image", frame);
        int key = cv::waitKey(1);
        if (key == 'q') break;
    }
}
