#include <chrono>

#include "arm.h"
#include "camera.h"
#include "aruco.h"
#include "control.h"
#include "compute.h"
#include "trajectory.h"

void delay(double s) {
    std::this_thread::sleep_for(std::chrono::microseconds((int) (s * 1000000)));
}

time_t getTime() {
    return std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
}

class CircleTrajectory: public Trajectory {
private:
    float R, Omega;

public:
    CircleTrajectory(float period, float t, float R): Trajectory(period, t), R(R) { Omega = 2 * M_PI / period; }
    ~CircleTrajectory() {}

    Vector3f GetPosition() {
        return Vector3f(R * cos(Omega * t), R * sin(Omega * t), 0.0);
    }

    Vector3f GetVelocity() {
        return Vector3f(-R * Omega * sin(Omega * t), R * Omega * cos(Omega * t), 0.0);
    }
};

class FourierTrajectory: public Trajectory {
private:
    float baseOmega;
    std::vector<float> magnitudes, phases, signs;

public:
    FourierTrajectory(float period, float t, std::vector<float>& magnitudes, std::vector<float>& phases, std::vector<float>& signs): Trajectory(period, t) {
        baseOmega = 2 * M_PI / period;
        this->magnitudes = magnitudes;
        this->phases = phases;
        this->signs = signs;
    }
    ~FourierTrajectory() {}

    Vector3f GetPosition() {
        float x = 0.0, y = 0.0, omega = baseOmega;
        for (size_t i = 0; i < magnitudes.size(); i ++) {
            x += magnitudes[i] * cos(signs[i] * omega * t + phases[i]);
            y += magnitudes[i] * sin(signs[i] * omega * t + phases[i]);
        }
        return Vector3f(x, y, 0.0);
    }

    Vector3f GetVelocity() {
        float vx = 0.0, vy = 0.0, omega = baseOmega;
        for (size_t i = 0; i < magnitudes.size(); i ++) {
            vx -= magnitudes[i] * signs[i] * omega * sin(signs[i] * omega * t + phases[i]);
            vy += magnitudes[i] * signs[i] * omega * cos(signs[i] * omega * t + phases[i]);
        }
    }
};

const float PEN_DOWN_DISTANCE = 0.1;

int main(int argc, char **argv) {
    // Arm
    std::shared_ptr<Gen3Lite> arm(new Gen3Lite());
    arm->MoveArmToHome();
    arm->SetGripperPosition(1.0);

    // Camera
    std::shared_ptr<RealSense> cam(new RealSense());
    Matrix3f R_0_E = quat2dcm(Quaternionf(0.0188425, -0.0246767, 0.968246, -0.248064)); // w, x, y, z
    Vector3f t_0_E = Vector3f(0.378836, 0.380062, 0.423619);
    Matrix4f T_0_E = Rt2T(R_0_E, t_0_E);

    // Aruco Markers
    ArucoDetector detector(cam, cv::aruco::DICT_4X4_50, 0.06);
    std::vector<ArucoMarker> markers;
    for (size_t i = 0; i <= 4; i ++) markers.push_back(ArucoMarker(i, 200));
    markers[1 - 1].SetT_M_P(Vector3f(0.0, 0.0, M_PI), Vector3f(0.16990917, 0.0975, 0.0));
    markers[2 - 1].SetT_M_P(Vector3f(0.0, 0.0, 2 * M_PI), Vector3f(0.17297744, 0.0975, 0.0));
    markers[3 - 1].SetT_M_P(Vector3f(0.0, 0.0, M_PI / 2), Vector3f(0.0975, 0.17248443, 0.0));
    markers[4 - 1].SetT_M_P(Vector3f(0.0, 0.0, M_PI), Vector3f(0.1755527, -0.0975, 0.0));
    ArucoMarker coMarker(-1);
    coMarker.SetT_M_P(Vector3f(0.0, 0.0, 2 * M_PI), Vector3f(0.0, 0.0, 0.0));

    // Controllers
    PIDController<Vector3f> linear_controller(1.6, 0.03, 0.4);      // 1.6, 0.01, 0.1
    PIDController<Vector2f> angular_controller(100.0, 1.0, -10.0);  // 80.0, 0.01, -5.0
    linear_controller.Init();
    angular_controller.Init();

    // Trajectory
    std::vector<float> magnitudes = {0.05, 0.04, 0.02, 0.016};
    std::vector<float> phases = {0.0, M_PI, M_PI / 2, -M_PI / 2};
    std::vector<float> signs = {-1.0, 1.0, -2.0, 2.0};
    FourierTrajectory trajectory(10.0, 0.0, magnitudes, phases, signs);

    // Main Loop
    int lost_frames = 0;
    time_t last_time = getTime(), current_time;
    Vector6f last_rtvec = coMarker.GetRtVec();

    try {
        while (true) {
            // Timing
            current_time = getTime();
            float dt = (current_time - last_time) / 1000000.0;
            last_time = current_time;
            // printf("%.2f fps\n", 1.0 / dt);

            // Video Capture
            cv::Mat frame = cam->GetColorFrame();

            // Aruco Detection
            std::vector<int> ids;
            std::vector<Vector3f> rvecs, tvecs;
            detector.Detect(frame, ids, rvecs, tvecs, false);
            // cv::imshow("Image", frame);
            // int key = cv::waitKey(1);
            // if (key == 'q') break;

            // Action
            if (!ids.empty()) {
                // Losting Detection
                lost_frames = 0;

                // Plane Detection
                Vector3f rvec_acc = Vector3f::Zero(), tvec_acc = Vector3f::Zero();
                for (size_t i = 0; i < ids.size(); i ++) {
                    int id = ids[i] - 1;
                    if (id >= 0 && id < 4) markers[id].Update(rvecs[i], tvecs[i]);
                    Vector6f rt_vec = markers[id].GetRtVec();
                    Matrix4f T_E_M = RtVec2T(rt_vec);
                    Matrix4f T_0_P_i = T_0_E * T_E_M * markers[id].GetT_M_P();
                    rvec_acc += dcm2rotvec(T_0_P_i.block<3, 3>(0, 0));
                    tvec_acc += T_0_P_i.block<3, 1>(0, 3);
                }
                rvec_acc /= ids.size();
                tvec_acc /= ids.size();
                coMarker.Update(rvec_acc, tvec_acc);
                Vector6f rtvec = coMarker.GetRtVec();
                last_rtvec = rtvec;
                Matrix4f T_0_P = RtVec2T(rtvec);

                // PID Controlling
                Vector6f current_pose = arm->GetToolCartesianPose();
                Vector6f current_twist = arm->GetToolTwist();
                
                Matrix4f T_0_H = pose2T(current_pose);
                Matrix4f T_H_0 = T_0_H.inverse();
                Matrix4f T_H_P = T_H_0 * T_0_P;

                Vector3f X_P_target = trajectory.GetPosition();
                X_P_target(2) += PEN_DOWN_DISTANCE;
                Vector3f X_H_target = (T_H_P * Vector4f(X_P_target(0), X_P_target(1), X_P_target(2), 1.0)).block<3, 1>(0, 0);

                Vector3f V_P_target = trajectory.GetVelocity();
                Vector3f V_H_target = Vector3f::Zero(); // T_H_P.block<3, 3>(0, 0) * V_P_target;

                Vector3f V_H_measure = T_H_0.block<3, 3>(0, 0) * current_twist.block<3, 1>(0, 0);
                Vector3f X_H_error = X_H_target; // - (0, 0, 0)
                Vector3f V_H_error = V_H_target - V_H_measure;
                Vector3f LinearTwist = linear_controller.Step(dt, X_H_error, V_H_error); // + 0.05 * X_H_target / dt;

                Matrix3f R_H_P = T_H_0.block<3, 3>(0, 0) * T_0_P.block<3, 3>(0, 0);
                Vector3f n_H = R_H_P.block<3, 1>(0, 2);
                Vector2f n_H_error = Vector2f(n_H(2), -n_H(0));
                Vector2f AngularTwist = angular_controller.Step(dt, n_H_error);

                Vector6f Twist;
                Twist << LinearTwist(0), LinearTwist(1), LinearTwist(2), AngularTwist(0), 0.0, AngularTwist(1);
                arm->SetTwist(Twist, k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
                
                // trajectory.Update(dt);
            } else {
                // Losting Detection
                lost_frames += 1;
                if (lost_frames > 10) {
                    arm->StopMotioning();
                }
            }
        }
    } catch (const std::exception& e) {
        arm->StopMotioning();
        arm->MoveArmToHome();
    }
    
    return 0;
}
