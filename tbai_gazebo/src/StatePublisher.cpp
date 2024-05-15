// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_gazebo/StatePublisher.hpp"

#include <functional>
#include <string>

#include <Eigen/Geometry>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/YamlConfig.hpp>
#include <tbai_msgs/RbdState.h>

#include "tbai_gazebo/legged_state_estimator/legged_state_estimator_settings.hpp"

namespace gazebo {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StatePublisher::Load(physics::ModelPtr robot, sdf::ElementPtr sdf) {
    ROS_INFO_STREAM("[StatePublisher] Loading StatePublisher plugin");
    // set Gazebo callback function
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&StatePublisher::OnUpdate, this));

    this->robot_ = robot;
    auto config = tbai::core::YamlConfig::fromRosParam("/tbai_config_path");

    ros::NodeHandle nh;
    auto stateTopic = config.get<std::string>("state_topic");
    statePublisher_ = nh.advertise<tbai_msgs::RbdState>(stateTopic, 2);

    auto base = config.get<std::string>("base_name");
    baseLinkPtr_ = robot->GetChildLink(base);

    // get joints; ignore 'universe' and 'root_joint'
    auto jointNames = config.get<std::vector<std::string>>("joint_names");
    for (int i = 0; i < jointNames.size(); ++i) {
        joints_.push_back(robot->GetJoint(jointNames[i]));
    }

    // initialize last publish time
    lastSimTime_ = robot->GetWorld()->SimTime();

    rate_ = config.get<double>("state_publisher/update_rate");
    period_ = 1.0 / rate_;

    // Setup contact flags - TODO(lnotspotl): This is a bit hacky, remove hardcoding!
    std::vector<std::string> contactTopics = {"/lf_foot_contact", "/rf_foot_contact", "/lh_foot_contact",
                                              "/rh_foot_contact"};
    for (int i = 0; i < contactTopics.size(); ++i) {
        contactFlags_[i] = false;
        auto callback = [this, i](const std_msgs::Bool::ConstPtr &msg) { contactFlags_[i] = msg->data; };
        contactSubscribers_[i] = nh.subscribe<std_msgs::Bool>(contactTopics[i], 1, callback);
    }

    // Initialize state estimator
    std::string urdfPath;
    ros::param::get("/description_file", urdfPath);
    std::cout << "[Debug] Loading state estimator with URDF: " << urdfPath << "\n";
    std::string urdf;
    ros::param::get("robot_description", urdf);
    auto settings = legged_state_estimator::LeggedStateEstimatorSettings::AnymalD(urdf, period_);
    settings.contact_estimator_settings.contact_force_covariance_alpha = 10.0;
    // settings.dynamic_contact_estimation = True
    settings.contact_position_noise = 0.1 ;;
    settings.contact_rotation_noise = 0.1 ;
    settings.lpf_gyro_accel_cutoff_frequency = 250;
    settings.lpf_lin_accel_cutoff_frequency  = 250;
    settings.lpf_dqJ_cutoff_frequency  = 10;
    settings.lpf_ddqJ_cutoff_frequency = 5;
    settings.lpf_tauJ_cutoff_frequency = 10;
    stateEstimatorPtr_ = std::make_unique<legged_state_estimator::LeggedStateEstimator>(settings);

    std::cout << "Done loading state estimator\n" << std::endl;
    ROS_INFO_STREAM("[StatePublisher] Loaded StatePublisher plugin");
}  // namespace gazebo

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StatePublisher::OnUpdate() {
    // Get current time
    const common::Time currentTime = robot_->GetWorld()->SimTime();
    const double dt = (currentTime - lastSimTime_).Double();

    // Check if update is needed
    if (dt < period_) {
        return;
    }

    // Unpack base pose
    const ignition::math::Pose3d &basePoseIgn = baseLinkPtr_->WorldPose();
    const auto &basePositionIgn = basePoseIgn.Pos();
    const auto &baseOrientationIgn = basePoseIgn.Rot();

    // Base orientation - Euler zyx
    const Eigen::Quaternion<double> baseQuaternion(baseOrientationIgn.W(), baseOrientationIgn.X(),
                                                   baseOrientationIgn.Y(), baseOrientationIgn.Z());
    const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
    const tbai::matrix3_t R_base_world = R_world_base.transpose();
    const tbai::vector_t rpy = tbai::core::mat2oc2rpy(R_world_base, lastYaw_);
    lastYaw_ = rpy[2];

    // Base position in world frame
    const Eigen::Vector3d basePosition(basePositionIgn.X(), basePositionIgn.Y(), basePositionIgn.Z());

    if (firstUpdate_) {
        lastOrientationBase2World_ = R_base_world;
        lastPositionBase_ = basePosition;
        firstUpdate_ = false;
    }

    // Base angular velocity in base frame
    const Eigen::Vector3d angularVelocityWorld = tbai::core::mat2aa(R_world_base * lastOrientationBase2World_) / dt;
    const Eigen::Vector3d angularVelocityBase = R_base_world * angularVelocityWorld;

    // Base linear velocity in base frame
    const Eigen::Vector3d linearVelocityWorld = (basePosition - lastPositionBase_) / dt;
    const Eigen::Vector3d linearVelocityBase = R_base_world * linearVelocityWorld;

    // Joint angles
    std::vector<double> jointAngles(joints_.size());
    for (int i = 0; i < joints_.size(); ++i) {
        jointAngles[i] = joints_[i]->Position(0);
    }

    // Joint velocities
    if (lastJointAngles_.size() != joints_.size()) {
        lastJointAngles_.resize(joints_.size());
        for (size_t i = 0; i < joints_.size(); ++i) {
            lastJointAngles_[i] = jointAngles[i];
        }
    }

    // Get joint velocities
    std::vector<double> jointVelocities(joints_.size());
    for (int i = 0; i < joints_.size(); ++i) {
        jointVelocities[i] = (jointAngles[i] - lastJointAngles_[i]) / dt;
        lastJointAngles_[i] = jointAngles[i];
    }

    // Put everything into an RbdState message
    tbai_msgs::RbdState message;  // TODO(lnotspotl): Room for optimization here

    // Base orientation - Euler zyx as {roll, pitch, yaw}
    message.rbd_state[0] = rpy[0];
    message.rbd_state[1] = rpy[1];
    message.rbd_state[2] = rpy[2];

    // Base position
    message.rbd_state[3] = basePosition[0];
    message.rbd_state[4] = basePosition[1];
    message.rbd_state[5] = basePosition[2];

    // Base angular velocity
    message.rbd_state[6] = angularVelocityBase[0];
    message.rbd_state[7] = angularVelocityBase[1];
    message.rbd_state[8] = angularVelocityBase[2];

    // Base linear velocity
    message.rbd_state[9] = linearVelocityBase[0];
    message.rbd_state[10] = linearVelocityBase[1];
    message.rbd_state[11] = linearVelocityBase[2];

    // Joint positions
    for (int i = 0; i < jointAngles.size(); ++i) {
        message.rbd_state[12 + i] = jointAngles[i];
    }

    // Joint velocities
    for (int i = 0; i < jointVelocities.size(); ++i) {
        message.rbd_state[12 + 12 + i] = jointVelocities[i];
    }

    // Observation time
    message.stamp = ros::Time::now();

    // Contact flags
    std::copy(contactFlags_.begin(), contactFlags_.end(), message.contact_flags.begin());

    lastOrientationBase2World_ = R_base_world;
    lastPositionBase_ = basePosition;

    // Publish message
    // statePublisher_.publish(message);

    lastSimTime_ = currentTime;


    // Data for state estimator
    using vector_t = Eigen::VectorXd;
    using vector3_t = Eigen::Vector3d;

    // TODO:
    // vector3_t imu_gyro_raw = {0.0, 0.0, 0.0};
    vector3_t imu_gyro_raw = angularVelocityBase;

    if(IMUFistUpdate_) {
        lastVelocityBaseIMU_ = linearVelocityBase;
        vector3_t basePositionIMU(basePosition[0], basePosition[1], basePosition[2]);
        stateEstimatorPtr_->init(basePositionIMU, baseQuaternion.coeffs(), linearVelocityWorld, vector3_t::Zero(), vector3_t::Zero());
        IMUFistUpdate_ = false;
    }

    imu_gyro_raw[1] *= 1.0;
    imu_gyro_raw[2] *= 1.0;

    vector3_t imu_lin_accel_raw = (linearVelocityBase - lastVelocityBaseIMU_) / dt;
    imu_lin_accel_raw = R_world_base * imu_lin_accel_raw;

    imu_lin_accel_raw[1] *= 1.0;
    imu_lin_accel_raw[2] *= 1.0;
    imu_lin_accel_raw[2] += 9.81;
    imu_lin_accel_raw = R_base_world * imu_lin_accel_raw;

    vector_t qJ(12);
    for(int i = 0; i < 12; i++) {
        qJ[i] = jointAngles[i];
    }

    vector_t dqJ(12);
    for(int i = 0; i < 12; i++) {
        dqJ[i] = jointVelocities[i];
    }

    std::array<bool, 4> contactFlags;
    for(int i = 0; i < 4; i++) {
        contactFlags[i] = contactFlags_[i];
    }

    // Update state estimator
    auto t1 = std::chrono::high_resolution_clock::now();
    stateEstimatorPtr_->update(imu_gyro_raw, imu_lin_accel_raw, qJ, dqJ, contactFlags);
    auto t2 = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1) / 1e3;
    std::cout << "State estimator update time: " << duration.count() << " ms" << std::endl;

    lastVelocityBaseIMU_ = linearVelocityBase;

    // Compute current and estimated rpy
    auto rpyEst = tbai::core::mat2rpy(stateEstimatorPtr_->getBaseRotationEstimate());
    auto rpyCur = tbai::core::mat2rpy(R_world_base);
    auto rpyEstOcs2 = tbai::core::mat2oc2rpy(stateEstimatorPtr_->getBaseRotationEstimate(), lastYawIMU_);
    lastYawIMU_ = rpyEstOcs2[2];

    std::cout << "Current rpy: " << rpyCur.transpose() << std::endl;
    std::cout << "Estimated rpy: " << rpyEst.transpose() << std::endl;

    // Compute estimated and current position
    auto posEst = stateEstimatorPtr_->getBasePositionEstimate();
    auto posCur = basePosition;

    std::cout << "Current position: " << posCur.transpose() << std::endl;
    std::cout << "Estimated position: " << posEst.transpose() << std::endl;
    std::cout << std::endl;

    auto angEst = stateEstimatorPtr_->getBaseAngularVelocityEstimateLocal();
    auto linEst = stateEstimatorPtr_->getBaseLinearVelocityEstimateLocal();

    // Base orientation - Euler zyx as {roll, pitch, yaw}
    message.rbd_state[0] = rpyEstOcs2[0];
    message.rbd_state[1] = rpyEstOcs2[1];
    message.rbd_state[2] = rpyEstOcs2[2];

    // Base position
    message.rbd_state[3] = posEst[0];
    message.rbd_state[4] = posEst[1];
    message.rbd_state[5] = posEst[2];

    // Base angular velocity
    message.rbd_state[6] = angEst[0];
    message.rbd_state[7] = angEst[1];
    message.rbd_state[8] = angEst[2];

    // Base linear velocity
    message.rbd_state[9] = linEst[0];
    message.rbd_state[10] = linEst[1];
    message.rbd_state[11] = linEst[2];

    statePublisher_.publish(message);
}

GZ_REGISTER_MODEL_PLUGIN(StatePublisher);

}  // namespace gazebo
