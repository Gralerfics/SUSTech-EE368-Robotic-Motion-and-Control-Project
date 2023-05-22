#include "arm.h"

#define CONNECTION_PORT 10000

constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};

Gen3Lite::Gen3Lite(CommanderArgs args) {
    // Create API objects
    auto error_callback = [](k_api::KError err){cout << "Callback error." << err.toString();};
    transport = std::make_shared<k_api::TransportClientTcp>();
    router = std::make_shared<k_api::RouterClient>(transport.get(), error_callback);
    transport->connect(args.ip_address, CONNECTION_PORT);

    // Set session data connection information
    auto session_info = k_api::Session::CreateSessionInfo();
    session_info.set_username(args.username);
    session_info.set_password(args.password);
    session_info.set_session_inactivity_timeout(60000);   // ms
    session_info.set_connection_inactivity_timeout(2000); // ms

    // Session manager service wrapper
    std::cout << "Creating session for communication..." << std::endl;
    session_manager = std::make_shared<k_api::SessionManager>(router.get());
    session_manager->CreateSession(session_info);
    std::cout << "Session created." << std::endl;

    // Create services
    base = std::make_shared<k_api::Base::BaseClient>(router.get());
    base_cyclic = std::make_shared<k_api::BaseCyclic::BaseCyclicClient>(router.get());

    // Start services
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

Gen3Lite::~Gen3Lite() {
    session_manager->CloseSession();
    router->SetActivationStatus(false);
    transport->disconnect();
}

std::function<void(k_api::Base::ActionNotification)> event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise) {
    return [&finish_promise] (k_api::Base::ActionNotification notification) {
        const auto action_event = notification.action_event();
        switch(action_event) {
            case k_api::Base::ActionEvent::ACTION_END:
                std::cout << "[Notification Event] ACTION_END." << std::endl;
                finish_promise.set_value(action_event);
                break;
            case k_api::Base::ActionEvent::ACTION_ABORT:
                std::cout << "[Notification Event] ACTION_ABORT." << std::endl;
                finish_promise.set_value(action_event);
                break;
            default:
                break;
        }
    };
}

bool Gen3Lite::wait_for_notification() {
    std::promise<k_api::Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
        event_listener_by_promise(finish_promise),
        k_api::Common::NotificationOptions()
    );
    const auto status = finish_future.wait_for(TIMEOUT_DURATION);
    base->Unsubscribe(promise_notification_handle);
    // const auto promise_event = finish_future.get();
    return status == std::future_status::ready;
}

k_api::BaseCyclic::Feedback Gen3Lite::GetFeedback() {
    auto feedback = base_cyclic->RefreshFeedback();
    return feedback;
}

Vector6f Gen3Lite::GetToolCartesianPose() {
    auto feedback = base_cyclic->RefreshFeedback();
    Vector6f pose;
    pose <<
        feedback.base().tool_pose_x(),
        feedback.base().tool_pose_y(),
        feedback.base().tool_pose_z(),
        feedback.base().tool_pose_theta_x(),
        feedback.base().tool_pose_theta_y(),
        feedback.base().tool_pose_theta_z();
    return pose;
}

Vector6f Gen3Lite::GetToolTwist() {
    auto feedback = base_cyclic->RefreshFeedback();
    Vector6f twist;
    twist <<
        feedback.base().tool_twist_linear_x(),
        feedback.base().tool_twist_linear_y(),
        feedback.base().tool_twist_linear_z(),
        feedback.base().tool_twist_angular_x(),
        feedback.base().tool_twist_angular_y(),
        feedback.base().tool_twist_angular_z();
    return twist;
}

bool Gen3Lite::MoveArmToHome() {
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list()) {
        if (action.name() == "Home") {
            action_handle = action.handle();
        }
    }
    if (action_handle.identifier() == 0) {
        std::cout << "[Error] Home cannot be reached." << std::endl;
        return false;
    }

    base->ExecuteActionFromReference(action_handle);

    bool finished = wait_for_notification();
    if (finished) {
        std::cout << "[Action] Succeed." << std::endl;
    } else {
        std::cout << "[Action] Failed." << std::endl;
    }
    return finished;
}

bool Gen3Lite::SetGripperPosition(float position) {
    k_api::Base::GripperCommand gripper_command;
    gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
    auto finger = gripper_command.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1);
    finger->set_value(position);

    base->SendGripperCommand(gripper_command);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));   // TODO
}

bool Gen3Lite::SetCartesianPose(Vector6f target_pose) {
    auto action = k_api::Base::Action();
    action.set_application_data("");

    auto constrained_pose = action.mutable_reach_pose();
    auto pose = constrained_pose->mutable_target_pose();
    pose->set_x(target_pose(0));
    pose->set_y(target_pose(1));
    pose->set_z(target_pose(2));
    pose->set_theta_x(target_pose(3));
    pose->set_theta_y(target_pose(4));
    pose->set_theta_z(target_pose(5));

    base->ExecuteAction(action);

    bool finished = wait_for_notification();
    if (finished) {
        std::cout << "[Action] Succeed." << std::endl;
    } else {
        std::cout << "[Action] Failed." << std::endl;
    }
    return finished;
}

void Gen3Lite::StopMotioning() {
    base->Stop();
}

void Gen3Lite::SetTwist(Vector6f target_twist, k_api::Common::CartesianReferenceFrame reference_frame) {
    auto command = k_api::Base::TwistCommand();
    command.set_reference_frame(reference_frame);
    command.set_duration(0);

    auto twist = command.mutable_twist();
    twist->set_linear_x(target_twist(0));
    twist->set_linear_y(target_twist(1));
    twist->set_linear_z(target_twist(2));
    twist->set_angular_x(target_twist(3));
    twist->set_angular_y(target_twist(4));
    twist->set_angular_z(target_twist(5));

    base->SendTwistCommand(command);
}
