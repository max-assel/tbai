
// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_mpc/MpcController.hpp"
#include "tbai_mpc/wbc/Factory.hpp"

#include <string>
#include <vector>

#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <tbai_core/Utils.hpp>
#include <tbai_core/Throws.hpp>

namespace tbai {
namespace mpc {

MpcController::MpcController(const std::shared_ptr<tbai::core::StateSubscriber> &stateSubscriberPtr)
    : stateSubscriberPtr_(stateSubscriberPtr), mrt_("anymal"), stopReferenceThread_(false) 
{
    std::cerr << "[MpcController] Initializing MPC controller" << std::endl;

    initTime_ = tbai::core::getEpochStart();

    const std::string robotName = "anymal";
    ros::NodeHandle nh;

    // Load default joint state
    // std::string targetCommandConfig;
    // TBAI_ROS_THROW_IF(!nh.getParam("/target_command_config_file", targetCommandConfig),
    //                   "Failed to get parameter /target_command_config_file");

    // URDF
    std::string urdfString;
    TBAI_ROS_THROW_IF(!nh.getParam("/robot_description", urdfString), "Failed to get parameter /robot_description");

    // Task settings
    std::string taskSettingsFile;
    TBAI_ROS_THROW_IF(!nh.getParam("/task_settings_file", taskSettingsFile),
                      "Failed to get parameter /task_settings_file");

    // Frame declarations
    std::string frameDeclarationFile;
    TBAI_ROS_THROW_IF(!nh.getParam("/frame_declaration_file", frameDeclarationFile),
                      "Failed to get parameter /frame_declaration_file");

    // Controller config
    std::string controllerConfigFile;
    TBAI_ROS_THROW_IF(!nh.getParam("/controller_config_file", controllerConfigFile),
                      "Failed to get parameter /controller_config_file");

    quadrupedInterfacePtr_ =
        anymal::getAnymalInterface(urdfString, switched_model::loadQuadrupedSettings(taskSettingsFile),
                                   anymal::frameDeclarationFromFile(frameDeclarationFile));

    visualizerPtr_ = std::make_unique<switched_model::QuadrupedVisualizer>(quadrupedInterfacePtr_->getKinematicModel(),
                                                                           quadrupedInterfacePtr_->getJointNames(),
                                                                           quadrupedInterfacePtr_->getBaseName(), nh);

    wbcPtr_ = switched_model::getWbcUnique(controllerConfigFile, urdfString, quadrupedInterfacePtr_->getComModel(),
                                           quadrupedInterfacePtr_->getKinematicModel(),
                                           quadrupedInterfacePtr_->getJointNames());

    referenceThreadNodeHandle_.setCallbackQueue(&referenceThreadCallbackQueue_);
    referenceTrajectoryGeneratorPtr_ = reference::getReferenceTrajectoryGeneratorUnique(referenceThreadNodeHandle_);

    mrt_.launchNodes(nh);
    tNow_ = 0.0;
}

void MpcController::spinOnceReferenceThread() 
{
    referenceThreadCallbackQueue_.callAvailable(ros::WallDuration(0.0));
}

tbai_msgs::JointCommandArray MpcController::getCommandMessage(scalar_t currentTime, scalar_t dt) 
{
    std::cerr << "[MpcController] getCommandMessage" << std::endl;

    mrt_.spinMRT();
    mrt_.updatePolicy();

    tNow_ = ros::Time::now().toSec() - initTime_;

    std::cerr << "  tNow_: " << tNow_ << std::endl;

    auto observation = generateSystemObservation();

    std::cerr << "  observation: " << std::endl;
    std::cerr << "    time: " << observation.time << std::endl;
    std::cerr << "    mode: " << observation.mode << std::endl;
    std::cerr << "    state: " << std::endl;
    std::cerr << "      torso orientation: " << observation.state.segment(0,3).transpose() << "\n" << std::endl;
    std::cerr << "      torso position:    " << observation.state.segment(3,3).transpose() << "\n" << std::endl;
    std::cerr << "      torso angular vel: " << observation.state.segment(6,3).transpose() << "\n" << std::endl;
    std::cerr << "      torso linear vel:  " << observation.state.segment(9,3).transpose() << "\n" << std::endl;
    std::cerr << "      LF joint pos:   " << observation.state.segment(12,3).transpose() << "\n" << std::endl;
    std::cerr << "      RF joint pos:   " << observation.state.segment(15,3).transpose() << "\n" << std::endl;
    std::cerr << "      LH joint pos:   " << observation.state.segment(18,3).transpose() << "\n" << std::endl;
    std::cerr << "      RH joint pos:   " << observation.state.segment(21,3).transpose() << "\n" << std::endl;
    std::cerr << "    input: " << std::endl;
    std::cerr << "      LF contact force: " << observation.input.segment(0,3).transpose() << "\n" << std::endl;
    std::cerr << "      RF contact force: " << observation.input.segment(3,3).transpose() << "\n" << std::endl;
    std::cerr << "      LH contact force: " << observation.input.segment(6,3).transpose() << "\n" << std::endl;
    std::cerr << "      RH contact force: " << observation.input.segment(9,3).transpose() << "\n" << std::endl;
    std::cerr << "      LF foot vel:      " << observation.input.segment(12,3).transpose() << "\n" << std::endl;
    std::cerr << "      RF foot vel:      " << observation.input.segment(15,3).transpose() << "\n" << std::endl;
    std::cerr << "      LH foot vel:      " << observation.input.segment(18,3).transpose() << "\n" << std::endl;
    std::cerr << "      RH foot vel:      " << observation.input.segment(21,3).transpose() << "\n" << std::endl;

    ocs2::vector_t desiredState;
    ocs2::vector_t desiredInput;
    size_t desiredMode;
    mrt_.evaluatePolicy(tNow_, observation.state, desiredState, desiredInput, desiredMode);

    std::cerr << "  desired: " << std::endl;
    std::cerr << "    mode: " << desiredMode << std::endl;
    std::cerr << "    state: " << std::endl;
    std::cerr << "      torso orientation: " << desiredState.segment(0,3).transpose() << "\n" << std::endl;
    std::cerr << "      torso position:    " << desiredState.segment(3,3).transpose() << "\n" << std::endl;
    std::cerr << "      torso angular vel: " << desiredState.segment(6,3).transpose() << "\n" << std::endl;
    std::cerr << "      torso linear vel:  " << desiredState.segment(9,3).transpose() << "\n" << std::endl;
    std::cerr << "      LF joint pos:      " << desiredState.segment(12,3).transpose() << "\n" << std::endl;
    std::cerr << "      RF joint pos:      " << desiredState.segment(15,3).transpose() << "\n" << std::endl;
    std::cerr << "      LH joint pos:      " << desiredState.segment(18,3).transpose() << "\n" << std::endl;
    std::cerr << "      RH joint pos:      " << desiredState.segment(21,3).transpose() << "\n" << std::endl;
    std::cerr << "    input: " << std::endl;
    std::cerr << "      LF contact force:  " << desiredInput.segment(0,3).transpose() << "\n" << std::endl;
    std::cerr << "      RF contact force:  " << desiredInput.segment(3,3).transpose() << "\n" << std::endl;
    std::cerr << "      LH contact force:  " << desiredInput.segment(6,3).transpose() << "\n" << std::endl;
    std::cerr << "      RH contact force:  " << desiredInput.segment(9,3).transpose() << "\n" << std::endl;
    std::cerr << "      LF foot vel:       " << desiredInput.segment(12,3).transpose() << "\n" << std::endl;
    std::cerr << "      RF foot vel:       " << desiredInput.segment(15,3).transpose() << "\n" << std::endl;
    std::cerr << "      LH foot vel:       " << desiredInput.segment(18,3).transpose() << "\n" << std::endl;
    std::cerr << "      RH foot vel:       " << desiredInput.segment(21,3).transpose() << "\n" << std::endl;

    constexpr ocs2::scalar_t time_eps = 1e-4;
    ocs2::vector_t dummyState;
    ocs2::vector_t dummyInput;
    size_t dummyMode;
    mrt_.evaluatePolicy(tNow_ + time_eps, observation.state, dummyState, dummyInput, dummyMode);

    std::cerr << "  dummy: " << std::endl;
    std::cerr << "    input: " << std::endl;
    std::cerr << "      LF contact force: " << desiredInput.segment(0,3).transpose() << "\n" << std::endl;
    std::cerr << "      RF contact force: " << desiredInput.segment(3,3).transpose() << "\n" << std::endl;
    std::cerr << "      LH contact force: " << desiredInput.segment(6,3).transpose() << "\n" << std::endl;
    std::cerr << "      RH contact force: " << desiredInput.segment(9,3).transpose() << "\n" << std::endl;
    std::cerr << "      LF foot vel:      " << desiredInput.segment(12,3).transpose() << "\n" << std::endl;
    std::cerr << "      RF foot vel:      " << desiredInput.segment(15,3).transpose() << "\n" << std::endl;
    std::cerr << "      LH foot vel:      " << desiredInput.segment(18,3).transpose() << "\n" << std::endl;
    std::cerr << "      RH foot vel:      " << desiredInput.segment(21,3).transpose() << "\n" << std::endl;

    ocs2::vector_t joint_accelerations = (dummyInput.tail<12>() - desiredInput.tail<12>()) / time_eps;

    std::cerr << "  joint accelerations: \n" << std::endl;
    std::cerr << "    LF_HAA acceleration: " << joint_accelerations[0] << "\n" << std::endl;
    std::cerr << "    LF_HFE acceleration: " << joint_accelerations[1] << "\n" << std::endl;
    std::cerr << "    LF_KFE acceleration: " << joint_accelerations[2] << "\n" << std::endl;
    std::cerr << "    RF_HAA acceleration: " << joint_accelerations[3] << "\n" << std::endl;
    std::cerr << "    RF_HFE acceleration: " << joint_accelerations[4] << "\n" << std::endl;
    std::cerr << "    RF_KFE acceleration: " << joint_accelerations[5] << "\n" << std::endl;
    std::cerr << "    LH_HAA acceleration: " << joint_accelerations[6] << "\n" << std::endl;
    std::cerr << "    LH_HFE acceleration: " << joint_accelerations[7] << "\n" << std::endl;
    std::cerr << "    LH_KFE acceleration: " << joint_accelerations[8] << "\n" << std::endl;
    std::cerr << "    RH_HAA acceleration: " << joint_accelerations[9] << "\n" << std::endl;
    std::cerr << "    RH_HFE acceleration: " << joint_accelerations[10] << "\n" << std::endl;
    std::cerr << "    RH_KFE acceleration: " << joint_accelerations[11] << "\n" << std::endl;

    auto commandMessage = wbcPtr_->getCommandMessage(tNow_, observation.state, observation.input, observation.mode,
                                                     desiredState, desiredInput, desiredMode, joint_accelerations);

    for (int i = 0; i < commandMessage.joint_commands.size(); i++)
    {
        std::cerr << "joint: " << commandMessage.joint_commands[i].joint_name << std::endl;
        std::cerr << "     desired position: " << commandMessage.joint_commands[i].desired_position << std::endl;
        std::cerr << "     desired velocity: " << commandMessage.joint_commands[i].desired_velocity << std::endl;
        std::cerr << "     kp:               " << commandMessage.joint_commands[i].kp << std::endl;
        std::cerr << "     kd:               " << commandMessage.joint_commands[i].kd << std::endl;
        std::cerr << "     torque_ff:        " << commandMessage.joint_commands[i].torque_ff << std::endl;
    }

    timeSinceLastMpcUpdate_ += dt;
    timeSinceLastVisualizationUpdate_ += dt;

    if (timeSinceLastMpcUpdate_ >= 1.0 / mpcRate_)
    {
        setObservation();
    }

    return commandMessage;
}

void MpcController::referenceThread() 
{
    // std::cerr << "MpcController::referenceThread" << std::endl;
    
    referenceTrajectoryGeneratorPtr_->reset();

    // Wait for initial mpc observation
    while (ros::ok() && !stopReferenceThread_) 
    {
        spinOnceReferenceThread();
        if (referenceTrajectoryGeneratorPtr_->isInitialized()) break;
        ros::Duration(0.02).sleep();
    }

    // Start reference thread
    ros::Rate rate(5.0);
    while (ros::ok() && !stopReferenceThread_) 
    {
        spinOnceReferenceThread();
        ROS_INFO_STREAM_THROTTLE(5.0, "[MpcController] Publishing reference");
        referenceTrajectoryGeneratorPtr_->publishReferenceTrajectory();
        rate.sleep();
    }
}

bool MpcController::checkStability() const {
    const auto &state = stateSubscriberPtr_->getLatestRbdState();
    scalar_t roll = state[0];
    if (roll >= 1.57 || roll <= -1.57) {
        return false;
    }
    scalar_t pitch = state[1];
    if (pitch >= 1.57 || pitch <= -1.57) {
        return false;
    }
    return true;
}

void MpcController::visualize() {
    if (timeSinceLastVisualizationUpdate_ >= 1.0 / 15.0) {
        visualizerPtr_->update(generateSystemObservation(), mrt_.getPolicy(), mrt_.getCommand());
        timeSinceLastVisualizationUpdate_ = 0.0;
    }
}

void MpcController::changeController(const std::string &controllerType, scalar_t currentTime) 
{
    // std::cerr << "[MpcController] changeController: " << controllerType << std::endl;

    if (!mrt_initialized_ || currentTime + 0.1 > mrt_.getPolicy().timeTrajectory_.back()) {
        resetMpc();
        mrt_initialized_ = true;
    }
    tNow_ = currentTime;

    // Start reference thread
    startReferenceThread();
}

void MpcController::startReferenceThread() 
{
    // std::cerr << "[MpcController] startReferenceThread" << std::endl;

    // Start reference thread
    if (referenceThread_.joinable()) {
        referenceThread_.join();
    }
    stopReferenceThread_ = false;
    referenceThread_ = std::thread(&MpcController::referenceThread, this);
}

void MpcController::stopReferenceThread() {
    stopReferenceThread_ = true;
}

bool MpcController::isSupported(const std::string &controllerType) {
    return controllerType == "WBC";
}

void MpcController::resetMpc() {
    // Generate initial observation
    stateSubscriberPtr_->waitTillInitialized();
    auto initialObservation = generateSystemObservation();
    const ocs2::TargetTrajectories initTargetTrajectories({0.0}, {initialObservation.state},
                                                          {initialObservation.input});
    mrt_.resetMpcNode(initTargetTrajectories);

    while (!mrt_.initialPolicyReceived() && ros::ok()) {
        ROS_INFO("Waiting for initial policy...");
        ros::spinOnce();
        mrt_.spinMRT();
        initialObservation = generateSystemObservation();
        mrt_.setCurrentObservation(initialObservation);
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("Initial policy received.");
}

void MpcController::setObservation() {
    mrt_.setCurrentObservation(generateSystemObservation());
    timeSinceLastMpcUpdate_ = 0.0;
}

ocs2::SystemObservation MpcController::generateSystemObservation() const 
{
    std::cerr << "[MpcController] generateSystemObservation" << std::endl;

    const tbai::vector_t &rbdState = stateSubscriberPtr_->getLatestRbdState();

    std::cerr << "  rbdState: " << std::endl;
    std::cerr << "     torso orientation (rpy):       " << rbdState.segment(0,3).transpose() << "\n" << std::endl;
    std::cerr << "     torso position (xyz):          " << rbdState.segment(3,3).transpose() << "\n" << std::endl;
    std::cerr << "     torso angular vel (base, rpy): " << rbdState.segment(6,3).transpose() << "\n" << std::endl;
    std::cerr << "     torso linear vel (base, xyz):  " << rbdState.segment(9,3).transpose() << "\n" << std::endl;
    std::cerr << "     LF joint positions:            " << rbdState.segment(12,3).transpose() << "\n" << std::endl;
    std::cerr << "     LH joint positions:            " << rbdState.segment(15,3).transpose() << "\n" << std::endl;
    std::cerr << "     RF joint positions:            " << rbdState.segment(18,3).transpose() << "\n" << std::endl;
    std::cerr << "     RH joint positions:            " << rbdState.segment(21,3).transpose() << "\n" << std::endl;
    std::cerr << "     LF joint velocity:             " << rbdState.segment(24,3).transpose() << "\n" << std::endl;
    std::cerr << "     LH joint velocity:             " << rbdState.segment(27,3).transpose() << "\n" << std::endl;
    std::cerr << "     RF joint velocity:             " << rbdState.segment(30,3).transpose() << "\n" << std::endl;
    std::cerr << "     RH joint velocity:             " << rbdState.segment(33,3).transpose() << "\n" << std::endl;

    // Set observation time
    ocs2::SystemObservation observation;
    observation.time = stateSubscriberPtr_->getLatestRbdStamp().toSec() - initTime_;

    // Set mode
    observation.mode = switched_model::stanceLeg2ModeNumber(stateSubscriberPtr_->getContactFlags());

    // Set state
    observation.state = rbdState.head<3 + 3 + 3 + 3 + 12>();

    // Swap LH and RF
    std::swap(observation.state(3 + 3 + 3 + 3 + 3 + 0), observation.state(3 + 3 + 3 + 3 + 3 + 3));
    std::swap(observation.state(3 + 3 + 3 + 3 + 3 + 1), observation.state(3 + 3 + 3 + 3 + 3 + 4));
    std::swap(observation.state(3 + 3 + 3 + 3 + 3 + 2), observation.state(3 + 3 + 3 + 3 + 3 + 5));

    // Set input
    observation.input.setZero(24);
    observation.input.tail<12>() = rbdState.tail<12>();

    // Swap LH and RF
    std::swap(observation.input(12 + 3), observation.input(12 + 6));
    std::swap(observation.input(12 + 4), observation.input(12 + 7));
    std::swap(observation.input(12 + 5), observation.input(12 + 8));

    return observation;
}

}  // namespace mpc
}  // namespace tbai
