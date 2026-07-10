/*
 * Argus Gate - argus_manager/src/manager_node.cpp
 * Copyright (c) 2026, Luca-G49
 * All rights reserved. Licensed under MIT License.
 */

#include "manager_node.hpp"

ArgusManagerNode::ArgusManagerNode()
: Node("argus_manager_node"),
  requested_mode_(ControlMode::IDLE),
  manager_state_(ManagerState::STOP),
  last_reset_error_(false),
  last_send_target_(false),
  last_target_pitch_(0.0f),
  last_target_yaw_(0.0f) {
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
    status_pub_ = this->create_publisher<argus_interfaces::msg::ManagerStatus>("manager_status", 10);

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

    ControlMode desired_mode = resolve_desired_mode(teleop_alive);
    bool ack_edge = detect_ack_edge();
    bool target_update = detect_target_update(desired_mode, teleop_alive);

    // --- STEP 2: STATE MACHINE (aligned with PLC states) ---
    ManagerState next_state = ManagerState::READY;

    if (!plc_alive) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLC Link Lost");
        next_state = ManagerState::READY;
    } else if (last_status_.error != 0) {
        next_state = ManagerState::ERROR;
        if (teleop_alive && ack_edge) {
            process_error_recovery(outbound_cmd);
            next_state = ManagerState::READY;
        }
    } else if (!teleop_alive) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Teleop Offline");
        next_state = ManagerState::READY;
    } else {
        switch (desired_mode) {
            case ControlMode::SYNCH:
                next_state = ManagerState::SYNCH;
                break;
            case ControlMode::JOG:
                next_state = ManagerState::JOG;
                break;
            case ControlMode::FOLLOW:
                next_state = ManagerState::FOLLOW;
                break;
            default:
                next_state = ManagerState::READY;
                break;
        }
    }

    bool state_entered = (manager_state_ != next_state);
    manager_state_ = next_state;

    requested_mode_ = (next_state == ManagerState::ERROR) ? ControlMode::IDLE : desired_mode;

    if (next_state == ManagerState::JOG) {
        execute_mode_logic(outbound_cmd, requested_mode_, state_entered, target_update);
    } else if (next_state == ManagerState::SYNCH) {
        execute_mode_logic(outbound_cmd, requested_mode_, state_entered, target_update);
    } else if (next_state == ManagerState::FOLLOW) {
        execute_mode_logic(outbound_cmd, requested_mode_, state_entered, target_update);
    }

    // Set the final mode mandated by ROS2
    outbound_cmd.mode = static_cast<int16_t>(requested_mode_);
    command_pub_->publish(outbound_cmd);

    // Update edge tracking for the next cycle.
    last_reset_error_ = last_teleop_cmd_.reset_error;
    last_send_target_ = last_teleop_cmd_.send_target;
    if (target_update) {
        last_target_pitch_ = last_teleop_cmd_.target_pitch;
        last_target_yaw_ = last_teleop_cmd_.target_yaw;
    }

    auto status_msg = argus_interfaces::msg::ManagerStatus();
    status_msg.state = state_to_string(manager_state_);
    status_msg.requested_mode = mode_to_string(requested_mode_);
    status_msg.teleop_online = teleop_alive;
    status_msg.plc_online = plc_alive;
    status_msg.error_active = (manager_state_ == ManagerState::ERROR) || (last_status_.error != 0);
    status_msg.busy = last_status_.busy;
    status_msg.done = last_status_.done;
    status_msg.synch = last_status_.synch;
    status_msg.on_target = last_status_.on_target;
    status_msg.pitch_position = last_status_.pos_pitch;
    status_msg.yaw_position = last_status_.pos_yaw;
    status_msg.target_pitch = last_teleop_cmd_.target_pitch;
    status_msg.target_yaw = last_teleop_cmd_.target_yaw;
    status_pub_->publish(status_msg);
}

ArgusManagerNode::ControlMode ArgusManagerNode::resolve_desired_mode(bool teleop_ready) const {
    if (!teleop_ready) {
        return ControlMode::IDLE;
    }

    switch (last_teleop_cmd_.requested_mode) {
        case 1:
            return ControlMode::SYNCH;
        case 2:
            return ControlMode::JOG;
        case 3:
            return ControlMode::FOLLOW;
        default:
            return ControlMode::IDLE;
    }
}

bool ArgusManagerNode::detect_ack_edge() const {
    return last_teleop_cmd_.reset_error && !last_reset_error_;
}

bool ArgusManagerNode::detect_target_update(ArgusManagerNode::ControlMode desired_mode, bool teleop_ready) const {
    if (!teleop_ready || desired_mode != ControlMode::FOLLOW) {
        return false;
    }

    bool send_target_edge = last_teleop_cmd_.send_target && !last_send_target_;
    bool target_changed = std::fabs(last_teleop_cmd_.target_pitch - last_target_pitch_) > 1e-6f
        || std::fabs(last_teleop_cmd_.target_yaw - last_target_yaw_) > 1e-6f;

    return send_target_edge || target_changed;
}

void ArgusManagerNode::execute_mode_logic(argus_interfaces::msg::PlcCommand &cmd,
                                          ArgusManagerNode::ControlMode desired_mode,
                                          bool state_entry,
                                          bool target_update) {
    switch (desired_mode) {
        case ControlMode::SYNCH:
            cmd.exec = 1;
            break;

        case ControlMode::JOG:
            cmd.exec = 0;
            map_teleop_to_jog(cmd);
            break;

        case ControlMode::FOLLOW:
            cmd.target_pitch = last_teleop_cmd_.target_pitch;
            cmd.target_yaw = last_teleop_cmd_.target_yaw;
            cmd.exec = 1;
            break;

        case ControlMode::IDLE:
            cmd.exec = 0;
        default:
            break;
    }
}

void ArgusManagerNode::map_teleop_to_jog(argus_interfaces::msg::PlcCommand &cmd) {
    // Map speed to jog directions.
    cmd.pitch_jog_p = (last_teleop_cmd_.pitch_speed > 0.0f);
    cmd.pitch_jog_n = (last_teleop_cmd_.pitch_speed < 0.0f);
    cmd.yaw_jog_p   = (last_teleop_cmd_.yaw_speed > 0.0f);
    cmd.yaw_jog_n   = (last_teleop_cmd_.yaw_speed < 0.0f);

    // Use full override unless the teleop layer provides a more specific scaling later.
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

std::string ArgusManagerNode::state_to_string(ManagerState state) const {
    switch (state) {
        case ManagerState::STOP:
            return "STOP";
        case ManagerState::READY:
            return "READY";
        case ManagerState::SYNCH:
            return "SYNCH";
        case ManagerState::JOG:
            return "JOG";
        case ManagerState::FOLLOW:
            return "FOLLOW";
        case ManagerState::ERROR:
        default:
            return "ERROR";
    }
}

std::string ArgusManagerNode::mode_to_string(ControlMode mode) const {
    switch (mode) {
        case ControlMode::SYNCH:
            return "SYNCH";
        case ControlMode::JOG:
            return "JOG";
        case ControlMode::FOLLOW:
            return "FOLLOW";
        case ControlMode::IDLE:
        default:
            return "IDLE";
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArgusManagerNode>());
    rclcpp::shutdown();
    return 0;
}
