#ifndef A1_CPP_A1ROBOTCONTROL_H
#define A1_CPP_A1ROBOTCONTROL_H

#include <iostream>
#include <string>
#include <chrono>

// ROS2-specific includes
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

// Other includes
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>

#include "A1Params.h"
#include "A1CtrlStates.h"
#include "utils/Utils.h"
#include "ConvexMpc.h"
#include "utils/filter.hpp"

class A1RobotControl {
public:
    A1RobotControl();
    A1RobotControl(std::shared_ptr<rclcpp::Node> node);

    void update_plan(A1CtrlStates &state, double dt);
    void generate_swing_legs_ctrl(A1CtrlStates &state, double dt);
    void compute_joint_torques(A1CtrlStates &state);
    Eigen::Matrix<double, 3, NUM_LEG> compute_grf(A1CtrlStates &state, double dt);
    Eigen::Vector3d compute_walking_surface(A1CtrlStates &state);

private:
    // Helpers and member variables
    BezierUtils bezierUtils[NUM_LEG];
    Eigen::Matrix<double, 6, 1> root_acc;
    Eigen::DiagonalMatrix<double, 6> Q;
    double R;
    double mu;
    double F_min;
    double F_max;
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    OsqpEigen::Solver solver;

    // ROS2-specific publisher and node handle members
    std::shared_ptr<rclcpp::Node> node_;
    std::array<rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr, NUM_LEG> pub_foot_start;
    std::array<rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr, NUM_LEG> pub_foot_end;
    std::array<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr, NUM_LEG> pub_foot_path;

    std::array<visualization_msgs::msg::Marker, NUM_LEG> foot_start_marker;
    std::array<visualization_msgs::msg::Marker, NUM_LEG> foot_end_marker;
    std::array<visualization_msgs::msg::Marker, NUM_LEG> foot_path_marker;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_terrain_angle;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_foot_pose_target_FL;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_foot_pose_target_FR;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_foot_pose_target_RL;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_foot_pose_target_RR;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_foot_pose_target_rel_FL;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_foot_pose_target_rel_FR;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_foot_pose_target_rel_RL;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_foot_pose_target_rel_RR;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_foot_pose_error_FL;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_foot_pose_error_FR;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_foot_pose_error_RL;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_foot_pose_error_RR;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_euler;

    int mpc_init_counter;
    std::string use_sim_time;
    MovingWindowFilter terrain_angle_filter;
    std::array<MovingWindowFilter, NUM_LEG> recent_contact_x_filter;
    std::array<MovingWindowFilter, NUM_LEG> recent_contact_y_filter;
    std::array<MovingWindowFilter, NUM_LEG> recent_contact_z_filter;
};

#endif //A1_CPP_A1ROBOTCONTROL_H
