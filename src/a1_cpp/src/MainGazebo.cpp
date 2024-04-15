#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "A1Params.h"
#include "GazeboA1ROS.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("gazebo_a1_qp_ctrl");

    // Change ROS logger level
    auto logger = node->get_logger();
    rclcpp::Logger rclcpp_logger = node->get_logger();
    rclcpp::Logger::set_level(rclcpp::Logger::Level::Debug);

    // Ensure ROS uses simulation time
    bool use_sim_time;
    node->get_parameter_or("use_sim_time", use_sim_time, false);
    if (!use_sim_time) {
        RCLCPP_ERROR(logger, "ROS must set use_sim_time in order to use this program!");
        return -1;
    }

    // Create A1 controller
    auto a1 = std::make_unique<GazeboA1ROS>(node);

    std::atomic<bool> control_execute{};
    control_execute.store(true);

    // Thread 1: compute desired ground forces
    RCLCPP_INFO(logger, "Enter thread 1: compute desired ground forces");
    std::thread compute_foot_forces_grf_thread([&]() {
        rclcpp::Time start = node->now();
        rclcpp::Time prev = start;
        rclcpp::Time now;
        rclcpp::Duration dt(0);

        while (control_execute.load() && rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(GRF_UPDATE_FREQUENCY));

            now = node->now();
            dt = now - prev;
            prev = now;

            auto t1 = std::chrono::high_resolution_clock::now();

            // Compute desired ground forces
            bool running = a1->update_foot_forces_grf(dt.seconds());

            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> ms_double = t2 - t1;
            RCLCPP_INFO(logger, "MPC solution is updated in %f ms", ms_double.count());

            if (!running) {
                RCLCPP_ERROR(logger, "Thread 1 loop is terminated because of errors.");
                rclcpp::shutdown();
                std::terminate();
                break;
            }
        }
    });

    // Thread 2: update robot states and send commands
    RCLCPP_INFO(logger, "Enter thread 2: update robot states and send commands");
    std::thread main_thread([&]() {
        rclcpp::Time start = node->now();
        rclcpp::Time prev = start;
        rclcpp::Time now;
        rclcpp::Duration dt(0);

        while (control_execute.load() && rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(MAIN_UPDATE_FREQUENCY));

            now = node->now();
            dt = now - prev;
            prev = now;

            // Compute desired swing legs forces and joint torques
            bool main_update_running = a1->main_update(node->now().seconds(), dt.seconds());
            bool send_cmd_running = a1->send_cmd();

            if (!main_update_running || !send_cmd_running) {
                RCLCPP_ERROR(logger, "Thread 2 loop is terminated because of errors.");
                rclcpp::shutdown();
                std::terminate();
                break;
            }
        }
    });

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    compute_foot_forces_grf_thread.join();
    main_thread.join();

    rclcpp::shutdown();
    return 0;
}
