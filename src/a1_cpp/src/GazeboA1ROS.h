#ifndef A1_CPP_GAZEBOA1ROS_H
#define A1_CPP_GAZEBOA1ROS_H

// std
#include <Eigen/Dense>
#include <memory>
#include <set>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fstream>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "ros2_unitree_legged_msgs/msg/motor_state.hpp"
#include "ros2_unitree_legged_msgs/msg/motor_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"

// control parameters
#include "A1Params.h"
#include "A1CtrlStates.h"
#include "A1RobotControl.h"
#include "A1BasicEKF.h"
#include "legKinematics/A1Kinematics.h"
#include "utils/Utils.h"
#include "utils/filter.hpp"

class GazeboA1ROS {
public:
    explicit GazeboA1ROS(std::shared_ptr<rclcpp::Node> node);

    bool update_foot_forces_grf(double dt);

    bool main_update(double t, double dt);

    bool send_cmd();

    // callback functions
    void gt_pose_callback(const nav_msgs::msg::Odometry::SharedPtr odom);

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu);

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

    void FL_hip_state_callback(const ros2_unitree_legged_msgs::msg::MotorState &a1_joint_state);

    void FL_thigh_state_callback(const ros2_unitree_legged_msgs::msg::MotorState &a1_joint_state);

    void FL_calf_state_callback(const ros2_unitree_legged_msgs::msg::MotorState &a1_joint_state);

    void FR_hip_state_callback(const ros2_unitree_legged_msgs::msg::MotorState &a1_joint_state);

    void FR_thigh_state_callback(const ros2_unitree_legged_msgs::msg::MotorState &a1_joint_state);

    void FR_calf_state_callback(const ros2_unitree_legged_msgs::msg::MotorState &a1_joint_state);

    void RL_hip_state_callback(const ros2_unitree_legged_msgs::msg::MotorState &a1_joint_state);

    void RL_thigh_state_callback(const ros2_unitree_legged_msgs::msg::MotorState &a1_joint_state);

    void RL_calf_state_callback(const ros2_unitree_legged_msgs::msg::MotorState &a1_joint_state);

    void RR_hip_state_callback(const ros2_unitree_legged_msgs::msg::MotorState &a1_joint_state);

    void RR_thigh_state_callback(const ros2_unitree_legged_msgs::msg::MotorState &a1_joint_state);

    void RR_calf_state_callback(const ros2_unitree_legged_msgs::msg::MotorState &a1_joint_state);

    void FL_foot_contact_callback(const geometry_msgs::msg::WrenchStamped &force);

    void FR_foot_contact_callback(const geometry_msgs::msg::WrenchStamped &force);

    void RL_foot_contact_callback(const geometry_msgs::msg::WrenchStamped &force);

    void RR_foot_contact_callback(const geometry_msgs::msg::WrenchStamped &force);

private:
    std::shared_ptr<rclcpp::Node> node_;

    // 0,  1,  2: FL_hip, FL_thigh, FL_calf
    // 3,  4,  5: FR_hip, FR_thigh, FR_calf
    // 6,  7,  8: RL_hip, RL_thigh, RL_calf
    // 9, 10, 11: RR_hip, RR_thigh, RR_calf
    std::array<rclcpp::Publisher<ros2_unitree_legged_msgs::msg::MotorCmd>::SharedPtr, 12> pub_joint_cmd;
    std::array<rclcpp::Subscription<ros2_unitree_legged_msgs::msg::MotorState>::SharedPtr, 12> sub_joint_msg;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_euler_d;

    // 0, 1, 2, 3: FL, FR, RL, RR
    std::array<rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr, 4> sub_foot_contact_msg;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gt_pose_msg;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_msg;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_msg;

    // debug estimation
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_estimated_pose;
    
    // Joystick commands and related variables
    double joy_cmd_velx = 0.0;
    double joy_cmd_velx_forward = 0.0;
    double joy_cmd_velx_backward = 0.0;
    double joy_cmd_vely = 0.0;
    double joy_cmd_velz = 0.0;

    double joy_cmd_pitch_rate = 0.0;
    double joy_cmd_roll_rate = 0.0;
    double joy_cmd_yaw_rate = 0.0;

    double joy_cmd_pitch_ang = 0.0;
    double joy_cmd_roll_ang = 0.0;
    double joy_cmd_body_height = 0.3;

    // 0 is standing, 1 is walking
    int joy_cmd_ctrl_state = 0;
    bool joy_cmd_ctrl_state_change_request = false;
    int prev_joy_cmd_ctrl_state = 0;
    bool joy_cmd_exit = false;

    // Robot kinematics and dynamics
    Eigen::Vector3d p_br;
    Eigen::Matrix3d R_br;
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    double motor_offset[4] = {};
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};
    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;
    A1Kinematics a1_kin;
    A1CtrlStates a1_ctrl_states;
    A1RobotControl _root_control;
    A1BasicEKF a1_estimate;

    // filters
    MovingWindowFilter acc_x;
    MovingWindowFilter acc_y;
    MovingWindowFilter acc_z;
    MovingWindowFilter gyro_x;
    MovingWindowFilter gyro_y;
    MovingWindowFilter gyro_z;
    MovingWindowFilter quat_w;
    MovingWindowFilter quat_x;
    MovingWindowFilter quat_y;
    MovingWindowFilter quat_z;
};


#endif //A1_CPP_GAZEBOA1ROS_H
