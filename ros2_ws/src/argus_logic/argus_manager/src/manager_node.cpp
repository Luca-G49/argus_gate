/*
 * Argus Gate - argus_manager/src/manager_node.cpp
 * Copyright (c) 2026, Name
 * All rights reserved. Licensed under MIT License.
 */

#include "manager_node.hpp"

ArgusManagerNode::ArgusManagerNode() : Node("argus_manager_node"), requested_mode_(ControlMode::IDLE) {
    // Initialize watchdog timers to "now"
    last_teleop_time_ = this->get_clock()->now();
    last_status_time_ = this->get_clock()->now();

    // --- 1. PARAMETERS ---
    this->declare_parameter("watchdog_timeout_ms", 500);
    this->declare_parameter("loop_frequency", 50.0);
    
    timeout_threshold_ = std::chrono::milliseconds(this->get_parameter("watchdog_timeout_ms").as_int());
    double freq = this->get_parameter("loop_frequency").as_double();

    // --- 2. COMMS ---
    teleop_sub_  = this->create_subscription<argus_interfaces::msg::TeleopCommand>(
        "teleop_cmd", 10, std::bind(&ArgusManagerNode::on_teleop_received, this, std::placeholders::_1));
    
    status_sub_  = this->create_subscription<argus_interfaces::msg::PlcStatus>(
        "plc_status", 10, std::bind(&ArgusManagerNode::on_status_received, this, std::placeholders::_1));
    
    command_pub_ = this->create_publisher<argus_interfaces::msg::PlcCommand>("plc_command", 10);

    // --- 3. MAIN CYCLE (50Hz) ---
    auto interval = std::chrono::duration<double>(1.0 / freq);
    timer_ = this->create_wall_timer(interval, std::bind(&ArgusManagerNode::supervisor_cycle, this));

    RCLCPP_INFO(this->get_logger(), "Argus Manager Node started (50Hz cycle)");
}

void ArgusManagerNode::on_teleop_received(const argus_interfaces::msg::TeleopCommand::SharedPtr msg) {
    last_teleop_cmd_ = *msg;
    last_teleop_time_ = this->get_clock()->now();
}

void ArgusManagerNode::on_status_received(const argus_interfaces::msg::PlcStatus::SharedPtr msg) {
    last_status_ = *msg;
    last_status_time_ = this->get_clock()->now();
}

void ArgusManagerNode::supervisor_cycle() {
    argus_interfaces::msg::PlcCommand outbound_cmd;
    reset_command(outbound_cmd);

    // --- STEP 1: QOS / HEALTH MONITORING ---
    bool teleop_alive = check_watchdog(last_teleop_time_, "Teleop");
    bool plc_alive = check_watchdog(last_status_time_, "PLC");

    if (!plc_alive) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLC Link Lost: Inhibiting commands");
        return; 
    }

    // --- STEP 2: MODE ARBITRATION ---
    // Update the requested mode based on UI/Teleop inputs
    update_requested_mode(teleop_alive);

    // Force IDLE mode if hardware reports an error
    if (last_status_.error != 0) {
        requested_mode_ = ControlMode::IDLE;
        process_error_recovery(outbound_cmd);
    } 
    else if (!teleop_alive && requested_mode_ == ControlMode::MANUAL_JOG) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Teleop Offline: Forcing Safe Stop");
        requested_mode_ = ControlMode::IDLE;
    } 
    else {
        // --- STEP 3: LOGIC EXECUTION ---
        execute_mode_logic(outbound_cmd);
    }

    // Set the final mode mandated by ROS2
    outbound_cmd.mode = static_cast<int16_t>(requested_mode_);
    command_pub_->publish(outbound_cmd);
}

void ArgusManagerNode::update_requested_mode(bool teleop_ready) {
    if (!teleop_ready) return;

    // Mode switching via Teleop requests
    if (last_teleop_cmd_.requested_mode == 0) requested_mode_ = ControlMode::IDLE;
    if (last_teleop_cmd_.requested_mode == 1) requested_mode_ = ControlMode::MANUAL_JOG;
    if (last_teleop_cmd_.requested_mode == 2) requested_mode_ = ControlMode::MANUAL_TRACK;
}

void ArgusManagerNode::execute_mode_logic(argus_interfaces::msg::PlcCommand &cmd) {
    switch (requested_mode_) {
        case ControlMode::MANUAL_JOG:
            map_teleop_to_jog(cmd);
            break;
        
        case ControlMode::MANUAL_TRACK:
            if (last_teleop_cmd_.send_target) {
                cmd.target_pitch = last_teleop_cmd_.target_pitch;
                cmd.target_yaw   = last_teleop_cmd_.target_yaw;
                cmd.exec = true; // Signal PLC to move to target
            }
            break;
        
        default:
            break;
    }
}

void ArgusManagerNode::map_teleop_to_jog(argus_interfaces::msg::PlcCommand &cmd) {
    // Map speed to jog directions
    cmd.pitch_jog_p = (last_teleop_cmd_.pitch_speed > 0);
    cmd.pitch_jog_n = (last_teleop_cmd_.pitch_speed < 0);
    cmd.yaw_jog_p   = (last_teleop_cmd_.yaw_speed > 0);
    cmd.yaw_jog_n   = (last_teleop_cmd_.yaw_speed < 0);
    // Overrides not in TeleopCommand, set to 100% or default
    cmd.pitch_override = 1.0f;
    cmd.yaw_override   = 1.0f;
}

void ArgusManagerNode::process_error_recovery(argus_interfaces::msg::PlcCommand &cmd) {
    cmd.ack = true; // Acknowledge error to reset PLC
}

void ArgusManagerNode::reset_command(argus_interfaces::msg::PlcCommand &cmd) {
    cmd = argus_interfaces::msg::PlcCommand(); // Reset to defaults
}

bool ArgusManagerNode::check_watchdog(const rclcpp::Time& last_time, const std::string& source) {
    auto elapsed = this->get_clock()->now() - last_time;
    return elapsed < timeout_threshold_;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArgusManagerNode>());
    rclcpp::shutdown();
    return 0;
}
