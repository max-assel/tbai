#pragma once

#include <vector>
#include <memory>

#include <Eigen/Dense>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tbai_core/Types.hpp>

#include "tbai_gazebo/legged_state_estimator/legged_state_estimator.hpp"

namespace gazebo {
class StatePublisher : public ModelPlugin {
   public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf);
    void OnUpdate();

   private:
    event::ConnectionPtr updateConnection_;

    /** RbdState message publisher */
    ros::Publisher statePublisher_;

    /** Robot gazebo model */
    physics::ModelPtr robot_;

    /** Base link */
    physics::LinkPtr baseLinkPtr_;

    std::vector<physics::JointPtr> joints_;

    /** State publish rate */
    double rate_;
    double period_;

    bool firstUpdate_ = true;

    double lastYaw_ = 0.0;
    double lastYawIMU_ = 0.0;

    std::array<bool, 4> contactFlags_;
    std::array<ros::Subscriber, 4> contactSubscribers_;

    // last yaw angle
    std::vector<tbai::scalar_t> lastJointAngles_;
    tbai::matrix3_t lastOrientationBase2World_;
    tbai::vector3_t lastPositionBase_;
    common::Time lastSimTime_;

    // State estimator
    std::unique_ptr<legged_state_estimator::LeggedStateEstimator> stateEstimatorPtr_;

    // Stuff to simulate the IMU
    bool IMUFistUpdate_ = true;
    tbai::matrix3_t lastOrientationBase2WorldIMU_;
    tbai::vector3_t lastPositionBaseIMU_;
    tbai::vector3_t lastVelocityBaseIMU_;
};

}  // namespace gazebo
