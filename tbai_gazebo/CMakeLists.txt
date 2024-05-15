cmake_minimum_required(VERSION 3.0.2)
project(tbai_gazebo)

## C++17 is a requirement
set (CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  realtime_tools
  tbai_msgs
  tbai_core
  ocs2_robotic_tools
  std_msgs
  pinocchio
)

find_package(gazebo REQUIRED)

catkin_package(
 CATKIN_DEPENDS tbai_msgs tbai_core
 DEPENDS pinocchio
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

## State estimator
add_library(StateEstimator
  src/legged_state_estimator/robot_model.cpp
  src/legged_state_estimator/legged_state_estimator.cpp
  src/legged_state_estimator/legged_state_estimator_settings.cpp
  src/legged_state_estimator/contact_estimator.cpp
  src/legged_state_estimator/inekf/inekf_state.cpp
  src/legged_state_estimator/inekf/inekf.cpp
  src/legged_state_estimator/inekf/lie_group.cpp
  src/legged_state_estimator/inekf/noise_params.cpp
  src/legged_state_estimator/inekf/observations.cpp
)
target_link_libraries(StateEstimator ${catkin_LIBRARIES})
target_compile_options(StateEstimator PRIVATE ${OCS2_CXX_FLAGS})

## Joint controller plugin
add_library(JointController src/JointController.cpp)
target_link_libraries(JointController ${catkin_LIBRARIES}) 
target_compile_options(JointController PRIVATE ${OCS2_CXX_FLAGS})

## RBD state plugin
# Anymal state estimator plugin
add_library(StatePublisher src/StatePublisher.cpp)
target_link_libraries(StatePublisher ${gazebo_LIBRARIES} ${catkin_LIBRARIES} StateEstimator)
target_compile_options(StatePublisher PRIVATE ${OCS2_CXX_FLAGS})

# Anymal contact sensor
add_library(ContactSensor SHARED src/ContactSensor.cpp)
target_link_libraries(ContactSensor ${catkin_LIBRARIES} ${gazebo_LIBRARIES})
target_compile_options(ContactSensor PRIVATE ${OCS2_CXX_FLAGS})