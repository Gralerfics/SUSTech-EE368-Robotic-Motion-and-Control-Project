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

const float POSITION_CHANGE_THRESHOLD = 0.05;
const float ANGULAR_CHANGE_THRESHOLD = 0.05; // TODO: Use Quaternion ?
const float ON_TARGET_EPS = 0.001;
const float PEN_UP_DISTANCE = 0.15;
const float PEN_DOWN_DISTANCE = 0.08;

enum TState { TAKEN_UP, TAKING_DOWN, DRAWING };

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
    for (size_t i = 0; i <= 4; i ++) markers.push_back(ArucoMarker(i));
    markers[1 - 1].SetT_M_P(Vector3f(0.0, 0.0, M_PI), Vector3f(0.16990917, 0.0975, 0.0));
    markers[2 - 1].SetT_M_P(Vector3f(0.0, 0.0, 2 * M_PI), Vector3f(0.17297744, 0.0975, 0.0));
    markers[3 - 1].SetT_M_P(Vector3f(0.0, 0.0, M_PI / 2), Vector3f(0.0975, 0.17248443, 0.0));
    markers[4 - 1].SetT_M_P(Vector3f(0.0, 0.0, M_PI), Vector3f(0.1755527, -0.0975, 0.0));
    ArucoMarker coMarker(-1);
    coMarker.SetT_M_P(Vector3f(0.0, 0.0, 2 * M_PI), Vector3f(0.0, 0.0, 0.0));

    // Controllers
    PIDController<Vector3f> linear_controller(1.6, 0.03, 0.2);
    PIDController<Vector2f> angular_controller(100.0, 1.0, -1.0);
    linear_controller.Init();
    angular_controller.Init();

    // State Machine and Trajectory
    TState state = TAKEN_UP;
    CircleTrajectory trajectory(4.0, 0.0, 0.08);

    // Main Loop
    int lost_frames = 0;
    time_t last_time, current_time;
    last_time = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    int stable_frames = 0;
    Vector6f last_rtvec = coMarker.GetRtVec();

    try {
        while (true) {
            // Timing
            current_time = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
            float dt = (current_time - last_time) / 1000000.0;
            last_time = current_time;
            printf("%.2f fps\n", 1.0 / dt);

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
                Vector3f delta_rvec = rtvec.head<3>() - last_rtvec.head<3>();
                Vector3f delta_tvec = rtvec.tail<3>() - last_rtvec.tail<3>();
                last_rtvec = rtvec;
                Matrix4f T_0_P = RtVec2T(rtvec);

                // PID Controlling
                Vector6f current_pose = arm->GetToolCartesianPose();
                Vector6f current_twist = arm->GetToolTwist();
                
                Matrix4f T_0_H = pose2T(current_pose);
                Matrix4f T_H_0 = T_0_H.inverse();
                Matrix4f T_H_P = T_H_0 * T_0_P;

                Vector3f X_P_target = trajectory.GetPosition();
                X_P_target(2) += (state == TAKEN_UP) ? PEN_UP_DISTANCE : PEN_DOWN_DISTANCE;
                Vector3f X_H_target = (T_H_P * Vector4f(X_P_target(0), X_P_target(1), X_P_target(2), 1.0)).block<3, 1>(0, 0);

                Vector3f V_P_target = trajectory.GetVelocity();
                Vector3f V_H_target = T_H_P.block<3, 3>(0, 0) * V_P_target;

                Vector3f V_H_measure = T_H_0.block<3, 3>(0, 0) * current_twist.block<3, 1>(0, 0);
                Vector3f X_H_error = X_H_target; // - (0, 0, 0)
                Vector3f V_H_error = V_H_target - V_H_measure;
                Vector3f LinearTwist = linear_controller.Step(dt, X_H_error, V_H_error);

                Matrix3f R_H_P = T_H_0.block<3, 3>(0, 0) * T_0_P.block<3, 3>(0, 0);
                Vector3f n_H = R_H_P.block<3, 1>(0, 2);
                Vector2f n_H_error = Vector2f(n_H(2), -n_H(0));
                Vector2f AngularTwist = angular_controller.Step(dt, n_H_error);

                Vector6f Twist;
                Twist << LinearTwist(0), LinearTwist(1), LinearTwist(2), AngularTwist(0), 0.0, AngularTwist(1);
                arm->SetTwist(Twist, k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);

                // Update State
                if (delta_rvec.norm() > ANGULAR_CHANGE_THRESHOLD || delta_tvec.norm() > POSITION_CHANGE_THRESHOLD) {
                    // Unstable
                    stable_frames = 0;
                    state = TAKEN_UP;
                } else {
                    // Stable
                    stable_frames += 1;
                    if (state == TAKEN_UP and stable_frames > 10) {
                        state = TAKING_DOWN;
                    } else if (state == TAKING_DOWN) {
                        if (X_H_error.norm() < ON_TARGET_EPS) { // 落笔到位
                            state = DRAWING;
                        }
                    } else {
                        trajectory.Update(dt);
                    }
                }
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
