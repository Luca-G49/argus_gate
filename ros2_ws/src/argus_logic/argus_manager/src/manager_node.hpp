/*
 * Argus Gate - argus_manager/src/manager_node.hpp
 * Copyright (c) 2026, Luca-G49
 * All rights reserved. Licensed under MIT License.
 */

#ifndef MANAGER_NODE_HPP
#define MANAGER_NODE_HPP

#include <memory>
#include <chrono>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include "argus_interfaces/msg/teleop_command.hpp"
#include "argus_interfaces/msg/plc_command.hpp"
#include "argus_interfaces/msg/plc_status.hpp"

using namespace std::chrono_literals;

/**
 * @class ArgusManagerNode
 * @brief Logic master: arbitrates system modes and ensures hardware safety interlocks.
 */
class ArgusManagerNode : public rclcpp::Node {
public:
    // Modes mandated by ROS2 to the PLC
    enum class ControlMode : int16_t  {
        IDLE = 0,
        MANUAL_JOG = 1,
        MANUAL_TRACK = 2,
        AUTO_TRACK = 5,
        ERROR_RESET = 99
    };

    ArgusManagerNode();
    ~ArgusManagerNode() = default;

private:
    void on_teleop_received(const argus_interfaces::msg::TeleopCommand::SharedPtr msg);
    void on_status_received(const argus_interfaces::msg::PlcStatus::SharedPtr msg);
    void supervisor_cycle();
    bool check_watchdog(const rclcpp::Time& last_time, const std::string& source);
    void publish_command();
    void update_requested_mode(bool teleop_ready);
    void execute_mode_logic(argus_interfaces::msg::PlcCommand &cmd);
    void map_teleop_to_jog(argus_interfaces::msg::PlcCommand &cmd);
    void process_error_recovery(argus_interfaces::msg::PlcCommand &cmd);
    void reset_command(argus_interfaces::msg::PlcCommand &cmd);

    // ROS2 Members
    rclcpp::Subscription<argus_interfaces::msg::TeleopCommand>::SharedPtr teleop_sub_;
    rclcpp::Subscription<argus_interfaces::msg::PlcStatus>::SharedPtr status_sub_;
    rclcpp::Publisher<argus_interfaces::msg::PlcCommand>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Watchdog timers
    rclcpp::Time last_teleop_time_;
    rclcpp::Time last_status_time_;
    std::chrono::milliseconds timeout_threshold_;

    // State
    ControlMode requested_mode_;
    argus_interfaces::msg::TeleopCommand last_teleop_cmd_;
    argus_interfaces::msg::PlcStatus last_status_;
};

#endif // MANAGER_NODE_HPP