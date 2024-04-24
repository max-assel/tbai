
#include <memory>

#include "ocs2_anymal_mpc/AnymalInterface.h"
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ros/ros.h>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/Controller.hpp>
#include <tbai_core/control/StateSubscriber.hpp>
#include <tbai_mpc/wbc/SqpWbc.hpp>
#include <tbai_msgs/JointCommandArray.h>
namespace tbai {

namespace mpc {
class MpcController final : public tbai::core::Controller {
   public:
    MpcController(const std::shared_ptr<tbai::core::StateSubscriber> &stateSubscriberPtr);

    tbai_msgs::JointCommandArray getCommandMessage(scalar_t currentTime, scalar_t dt) override;

    void visualize() override;

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

    bool isSupported(const std::string &controllerType) override;

    scalar_t getRate() const override { return 200.0; }

   private:
    std::shared_ptr<tbai::core::StateSubscriber> stateSubscriberPtr_;

    std::unique_ptr<switched_model::QuadrupedInterface> quadrupedInterfacePtr_;
    std::shared_ptr<switched_model::QuadrupedVisualizer> visualizerPtr_;
    std::unique_ptr<switched_model::SqpWbc> wbcPtr_;

    void resetMpc();

    void setObservation();

    ocs2::SystemObservation generateSystemObservation() const;

    scalar_t initTime_;
    scalar_t tNow_;

    ocs2::MRT_ROS_Interface mrt_;
    bool mrt_initialized_ = false;

    scalar_t mpcRate_ = 30.0;
    scalar_t timeSinceLastMpcUpdate_ = 1e5;
};

}  // namespace mpc
}  // namespace tbai