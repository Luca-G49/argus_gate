/*
 * Argus Gate - argus_manager/src/manager_node.hpp
 * Copyright (c) 2026, Luca-G49
 * All rights reserved. Licensed under MIT License.
 */

#ifndef MANAGER_NODE_HPP
#define MANAGER_NODE_HPP

#include <memory>
#include <chrono>
#include <cmath>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include "argus_interfaces/msg/teleop_command.hpp"
#include "argus_interfaces/msg/plc_command.hpp"
#include "argus_interfaces/msg/plc_status.hpp"
#include "argus_interfaces/msg/manager_status.hpp"

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
        SYNCH = 1,
        JOG = 2,
        FOLLOW = 3
    };

    enum class ManagerState : int8_t {
        STOP = 0,
        READY = 1,
        SYNCH = 2,
        JOG = 3,
        FOLLOW = 4,
        ERROR = 5
    };

    ArgusManagerNode();
    ~ArgusManagerNode() = default;

private:
    void on_teleop_received(const argus_interfaces::msg::TeleopCommand::SharedPtr msg);
    void on_status_received(const argus_interfaces::msg::PlcStatus::SharedPtr msg);
    void supervisor_cycle();
    bool check_watchdog(const rclcpp::Time& last_time, const std::string& source);
    std::string state_to_string(ManagerState state) const;
    std::string mode_to_string(ControlMode mode) const;
    ControlMode resolve_desired_mode(bool teleop_ready) const;
    bool detect_ack_edge() const;
    bool detect_target_update(ControlMode desired_mode, bool teleop_ready) const;
    void execute_mode_logic(argus_interfaces::msg::PlcCommand &cmd, ControlMode desired_mode, bool state_entry, bool target_update);
    void map_teleop_to_jog(argus_interfaces::msg::PlcCommand &cmd);
    void process_error_recovery(argus_interfaces::msg::PlcCommand &cmd);
    void reset_command(argus_interfaces::msg::PlcCommand &cmd);

    // ROS2 Members
    rclcpp::Subscription<argus_interfaces::msg::TeleopCommand>::SharedPtr teleop_sub_;
    rclcpp::Subscription<argus_interfaces::msg::PlcStatus>::SharedPtr status_sub_;
    rclcpp::Publisher<argus_interfaces::msg::PlcCommand>::SharedPtr command_pub_;
    rclcpp::Publisher<argus_interfaces::msg::ManagerStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Watchdog timers
    rclcpp::Time last_teleop_time_;
    rclcpp::Time last_status_time_;
    std::chrono::milliseconds timeout_threshold_;

    // State
    ControlMode requested_mode_;
    ManagerState manager_state_;
    bool last_reset_error_;
    bool last_send_target_;
    float last_target_pitch_;
    float last_target_yaw_;
    argus_interfaces::msg::TeleopCommand last_teleop_cmd_;
    argus_interfaces::msg::PlcStatus last_status_;
};

#endif // MANAGER_NODE_HPP