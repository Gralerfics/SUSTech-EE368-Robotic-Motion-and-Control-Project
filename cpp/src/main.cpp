#include <chrono>

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

    ArucoDetector detector(cam, cv::aruco::DICT_4X4_50, 0.06);
    std::vector<ArucoMarker> markers;
    for (size_t i = 0; i <= 4; i ++) markers.push_back(ArucoMarker(i));
    markers[1 - 1].SetT_M_P(Vector3f(0.0, 0.0, M_PI), Vector3f(0.16990917, 0.0975, 0.0));
    markers[2 - 1].SetT_M_P(Vector3f(0.0, 0.0, 2 * M_PI), Vector3f(0.17297744, 0.0975, 0.0));
    markers[3 - 1].SetT_M_P(Vector3f(0.0, 0.0, M_PI / 2), Vector3f(0.0975, 0.17248443, 0.0));
    markers[4 - 1].SetT_M_P(Vector3f(0.0, 0.0, M_PI), Vector3f(0.1755527, -0.0975, 0.0));
    ArucoMarker coMarker(-1);
    coMarker.SetT_M_P(Vector3f(0.0, 0.0, 2 * M_PI), Vector3f(0.0, 0.0, 0.0));

    PIDController<Vector3f> linear_controller(1.6, 0.03, 0.2);
    PIDController<Vector2f> angular_controller(100.0, 1.0, -1.0);

    Matrix3f R_0_E = quat2dcm(Quaternionf(0.0188425, -0.0246767, 0.968246, -0.248064)); // w, x, y, z
    Vector3f t_0_E = Vector3f(0.378836, 0.380062, 0.423619);
    Matrix4f T_0_E = Rt2T(R_0_E, t_0_E);

    arm->MoveArmToHome();
    arm->SetGripperPosition(1.0);

    linear_controller.Init();
    angular_controller.Init();

    float Phi = 0.0;
    float R = 0.08;
    float Omega = 2 * M_PI / 4;

    int lost_frames = 0;
    time_t last_time, current_time;
    last_time = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    while (true) {
        current_time = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
        float dt = (current_time - last_time) / 1000000.0;
        last_time = current_time;
        printf("%.2f fps\n", 1.0 / dt);

        cv::Mat frame = cam->GetColorFrame();

        std::vector<int> ids;
        std::vector<Vector3f> rvecs, tvecs;
        detector.Detect(frame, ids, rvecs, tvecs, true);

        cv::imshow("Image", frame);
        int key = cv::waitKey(1);
        if (key == 'q') break;

        if (!ids.empty()) {
            lost_frames = 0;

            Vector3f rvec_acc = Vector3f::Zero(), tvec_acc = Vector3f::Zero();
            for (size_t i = 0; i < ids.size(); i ++) {
                int id = ids[i] - 1;
                if (id >= 0 && id < 4) {
                    markers[id].Update(rvecs[i], tvecs[i]);
                }
                Vector6f rt_vec = markers[id].GetRtVec();
                Matrix4f T_E_M = RtVec2T(rt_vec);
                Matrix4f T_0_P_i = T_0_E * T_E_M * markers[id].GetT_M_P();
                rvec_acc += dcm2rotvec(T_0_P_i.block<3, 3>(0, 0));
                tvec_acc += T_0_P_i.block<3, 1>(0, 3);
            }
            rvec_acc /= ids.size();
            tvec_acc /= ids.size();
            coMarker.Update(rvec_acc, tvec_acc);
            Vector6f rt_vec_P = coMarker.GetRtVec();
            Matrix4f T_0_P = RtVec2T(rt_vec_P);

            Vector6f current_pose = arm->GetToolCartesianPose();
            Vector6f current_twist = arm->GetToolTwist();
            
            Matrix4f T_0_H = pose2T(current_pose);
            Matrix4f T_H_0 = T_0_H.inverse();

            Phi += Omega * dt;
            Vector4f X_P_target = Vector4f(R * cos(Phi), R * sin(Phi), 0.08, 1.0);
            Vector3f X_H_target = (T_H_0 * T_0_P * Vector4f(X_P_target(0), X_P_target(1), X_P_target(2), 1.0)).block<3, 1>(0, 0);

            // ...
            Vector3f V_H_target = Vector3f(0.0, 0.0, 0.0);

            Vector3f V_H_measure = T_H_0.block<3, 3>(0, 0) * current_twist.block<3, 1>(0, 0);
            Vector3f X_H_error = X_H_target;
            Vector3f V_H_error = V_H_target - V_H_measure;
            Vector3f LinearTwist = linear_controller.Step(dt, X_H_error, V_H_error);

            Matrix3f R_H_P = T_H_0.block<3, 3>(0, 0) * T_0_P.block<3, 3>(0, 0);
            Vector3f n_H = R_H_P.block<3, 1>(0, 2);
            Vector2f n_H_error = Vector2f(n_H(2), -n_H(0));
            Vector2f AngularTwist = angular_controller.Step(dt, n_H_error);

            Vector6f Twist;
            Twist << LinearTwist(0), LinearTwist(1), LinearTwist(2), AngularTwist(0), 0.0, AngularTwist(1);
            arm->SetTwist(Twist, k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
        } else {
            lost_frames += 1;
            if (lost_frames > 10) {
                arm->StopMotioning();
            }
        }
    }

    arm->StopMotioning();
    arm->MoveArmToHome();
}
