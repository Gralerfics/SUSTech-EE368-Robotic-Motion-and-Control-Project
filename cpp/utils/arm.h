#ifndef ARM_H
#define ARM_H

#include <memory>

#include "compute.h"

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include "connect_utilities.h"
namespace k_api = Kinova::Api;

class Gen3Lite {
private:
    std::shared_ptr<k_api::TransportClientTcp> transport = nullptr;
    std::shared_ptr<k_api::RouterClient> router = nullptr;
    std::shared_ptr<k_api::SessionManager> session_manager = nullptr;
    std::shared_ptr<k_api::Base::BaseClient> base = nullptr;
    std::shared_ptr<k_api::BaseCyclic::BaseCyclicClient> base_cyclic = nullptr;

    bool wait_for_notification();

public:
    Gen3Lite(CommanderArgs args = CommanderArgs{"192.168.1.10", "admin", "admin"});
    ~Gen3Lite();

    k_api::BaseCyclic::Feedback GetFeedback();
    Vector6f GetToolCartesianPose();
    Vector6f GetToolTwist();
    // Vector6f GetJointAngles();
    // Vector6f GetAngularVelocity();

    bool MoveArmToHome();
    bool SetGripperPosition(float position);
    bool SetCartesianPose(Vector6f pose);
    void StopMotioning();
    void SetTwist(Vector6f twist, k_api::Common::CartesianReferenceFrame reference_frame = k_api::Common::CARTESIAN_REFERENCE_FRAME_BASE);
};

#endif