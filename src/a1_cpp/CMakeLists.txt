cmake_minimum_required(VERSION 3.10.1)
project(a1_cpp)

# Set C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags
add_compile_options(-Wall -Wextra -Wpedantic -O3 -g)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  /usr/local/include/osqp  # for osqp.h
  $ENV{UNITREE_LEGGED_SDK_PATH}/include  # Unitree hardware library
)

# Link directories
# link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
# set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

# Library declaration
add_library(a1_lib
  src/legKinematics/A1Kinematics.h
  src/legKinematics/A1Kinematics.cpp
  src/A1Params.h
  src/A1CtrlStates.h
  src/utils/Utils.cpp
  src/utils/Utils.h
  src/A1RobotControl.cpp
  src/A1RobotControl.h
  src/A1BasicEKF.cpp
  src/A1BasicEKF.h
  src/ConvexMpc.cpp
  src/ConvexMpc.h
)
ament_target_dependencies(a1_lib rclcpp std_msgs geometry_msgs OsqpEigen ros2_unitree_legged_msgs)

add_library(gazebo_a1_lib
        src/GazeboA1ROS.h
        src/GazeboA1ROS.cpp
)
target_link_libraries(gazebo_a1_lib a1_lib)

# Executables
# add_executable(hardware_a1_ctrl src/MainHardware.cpp)
# target_link_libraries(hardware_a1_ctrl a1_lib ${EXTRA_LIBS})
# ament_target_dependencies(hardware_a1_ctrl rclcpp std_msgs geometry_msgs)

add_executable(gazebo_a1_ctrl src/MainGazebo.cpp)
target_link_libraries(gazebo_a1_ctrl
        gazebo_a1_lib
        OsqpEigen::OsqpEigen
)

# Installation
install(TARGETS
  a1_lib
  gazebo_a1_lib
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# If you have other executables or libraries, repeat the process above for each.

# Setup testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_package()
